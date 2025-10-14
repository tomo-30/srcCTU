# include/robot_client.py
import socket, select, time, threading
from typing import Tuple
from .math_utils import rpy_deg_to_quat, fmt
from info.config import (DEFAULT_RECV_TIMEOUT, IDLE_FRAME_WINDOW_SEC,
                         MOVEL_ACK_TIMEOUT_SEC)

class RecoverableCommError(Exception):
    pass

class RobotClient:
    def __init__(self, host, port):
        self.host = host; self.port = port
        self.sock: socket.socket | None = None
        self._sock_lock = threading.Lock()
        self._rxbuf = b""
        self._drain_stop = threading.Event()
        self._drain_thread: threading.Thread | None = None

    # --- connect/close ---
    def connect(self, timeout=5.0):
        self.sock = socket.create_connection((self.host, self.port), timeout=timeout)
        self.sock.settimeout(DEFAULT_RECV_TIMEOUT)
        print(f"[INFO] Connected {self.host}:{self.port}")

    def close(self):
        try: self.stop_ack_drain()
        except: pass
        try:
            if self.sock: self.sock.close()
        finally:
            self.sock = None

    # --- framed/idle recv ---
    def _recv_frame(self, deadline: float) -> str:
        while True:
            idx = self._rxbuf.find(b'#')
            if idx != -1:
                frame = self._rxbuf[:idx].decode("ascii", errors="ignore").strip()
                self._rxbuf = self._rxbuf[idx+1:]
                if frame: return frame

            now = time.time()
            if now > deadline:
                if self._rxbuf:
                    frame = self._rxbuf.decode("ascii", errors="ignore").strip()
                    self._rxbuf = b""
                    if frame: return frame
                raise TimeoutError("recv timeout while waiting for '#'")

            wait = min(IDLE_FRAME_WINDOW_SEC, max(0.0, deadline - now))
            r, _, _ = select.select([self.sock], [], [], wait)
            if r:
                with self._sock_lock:
                    chunk = self.sock.recv(4096)
                if chunk:
                    self._rxbuf += chunk
                    continue
            if self._rxbuf:
                frame = self._rxbuf.decode("ascii", errors="ignore").strip()
                self._rxbuf = b""
                if frame: return frame

    # --- ack drain ---
    def start_ack_drain(self):
        if not self.sock: raise RuntimeError("Socket not connected")
        if self._drain_thread and self._drain_thread.is_alive(): return
        with self._sock_lock:
            self.sock.setblocking(False)
        self._drain_stop.clear()
        self._drain_thread = threading.Thread(target=self._drain_loop, daemon=True)
        self._drain_thread.start()

    def _drain_loop(self):
        while not self._drain_stop.is_set():
            try:
                with self._sock_lock:
                    if not self.sock: break
                    _ = self.sock.recv(4096)
                time.sleep(0.002)
            except (BlockingIOError, socket.timeout):
                time.sleep(0.004)
            except Exception:
                time.sleep(0.02)

    def stop_ack_drain(self):
        if not self._drain_thread: return
        self._drain_stop.set()
        self._drain_thread.join(timeout=0.5)
        self._drain_thread = None
        with self._sock_lock:
            if self.sock:
                self.sock.setblocking(True)
                self.sock.settimeout(DEFAULT_RECV_TIMEOUT)
        time.sleep(0.03)
        print("[INFO] ACK drain stopped.")

    def flush_recv(self):
        if not self.sock: return
        self._rxbuf = b""
        with self._sock_lock:
            old_to = self.sock.gettimeout()
            self.sock.setblocking(False)
            total = 0
            try:
                while True:
                    try:
                        chunk = self.sock.recv(4096)
                        if not chunk: break
                        total += len(chunk)
                    except (BlockingIOError, socket.timeout):
                        break
            finally:
                self.sock.setblocking(True)
                self.sock.settimeout(old_to)
        if total: print(f"[INFO] Flushed {total} bytes.")

    # --- common send/recv ---
    def _send_and_recv(self, parts, recv_timeout=None, expect=None,
                       pause=0.12, allow_stray_move_ack=False):
        if not self.sock: raise RuntimeError("Socket not connected")
        msg = " ".join(fmt(p) for p in parts) + " #"
        with self._sock_lock:
            self.sock.sendall(msg.encode("ascii"))
        time.sleep(pause)

        deadline = time.time() + (recv_timeout if recv_timeout else DEFAULT_RECV_TIMEOUT)
        last_raw = ""
        while True:
            try:
                raw = self._recv_frame(deadline)
            except TimeoutError as te:
                raise RecoverableCommError(str(te)) from te

            last_raw = raw
            print(f"[SEND] {msg.strip()}   [RECV] {raw}")
            toks = raw.split()
            instr = int(toks[0]) if len(toks) >= 1 and toks[0].lstrip("-").isdigit() else None
            ok    = int(toks[1]) if len(toks) >= 2 and toks[1].lstrip("-").isdigit() else None

            if instr is None or ok is None:
                continue
            if (expect is None) or (instr == expect):
                return instr, ok, toks
            if allow_stray_move_ack and instr == 1:
                continue

            raise RecoverableCommError(
                f"ACK instr mismatch: expected {expect}, got {instr}, raw='{last_raw}'"
            )

    # --- wrappers (open_abb CASEs) ---
    def ping(self): return self._send_and_recv([0], expect=0, allow_stray_move_ack=True)
    def set_tool(self, txyz, rpy_deg):
        q = rpy_deg_to_quat(*rpy_deg)
        return self._send_and_recv([6, *txyz, *q], expect=6, allow_stray_move_ack=True)
    def set_wobj(self, identity=True, txyz=(0,0,0), rpy_deg=(0,0,0)):
        if identity:
            return self._send_and_recv([7, 0.0,0.0,0.0, 1.0,0.0,0.0,0.0], expect=7, allow_stray_move_ack=True)
        else:
            q = rpy_deg_to_quat(*rpy_deg)
            return self._send_and_recv([7, *txyz, *q], expect=7, allow_stray_move_ack=True)
    def set_speed(self, v_tcp, v_ori):
        return self._send_and_recv([8, v_tcp, v_ori], expect=8, allow_stray_move_ack=True)
    def set_zone(self, p_tcp, p_ori, z_ori):
        return self._send_and_recv([9, 0, p_tcp, p_ori, z_ori], expect=9, allow_stray_move_ack=True)

    def get_cart(self):
        instr, ok, toks = self._send_and_recv([3], expect=3, allow_stray_move_ack=True)
        if ok != 1 or len(toks) < 9:
            raise RuntimeError(f"CASE3 reply invalid: '{' '.join(toks)}'")
        x = float(toks[2]); y = float(toks[3]); z = float(toks[4])
        qx = float(toks[5]); qy = float(toks[6]); qz = float(toks[7]); qw = float(toks[8])
        return (x, y, z, qx, qy, qz, qw)

    def moveL_ack(self, x, y, z, rpy_deg, ack_timeout=MOVEL_ACK_TIMEOUT_SEC):
        qx, qy, qz, qw = rpy_deg_to_quat(*rpy_deg)
        return self._send_and_recv([1, x, y, z, qx, qy, qz, qw],
                                   recv_timeout=ack_timeout, expect=1)

    def moveL_send_only(self, x, y, z, rpy_deg, tiny_pause=0.03):
        if not self.sock: raise RuntimeError("Socket not connected")
        qx, qy, qz, qw = rpy_deg_to_quat(*rpy_deg)
        msg = " ".join(fmt(p) for p in [1, x, y, z, qx, qy, qz, qw]) + " #"
        with self._sock_lock:
            self.sock.sendall(msg.encode("ascii"))
        time.sleep(tiny_pause)
