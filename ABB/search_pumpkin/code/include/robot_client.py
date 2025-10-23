# include/robot_client.py
import socket, select, time, threading, collections
from typing import Tuple
from .math_utils import rpy_deg_to_quat, fmt
from info.config import (DEFAULT_RECV_TIMEOUT, IDLE_FRAME_WINDOW_SEC,
                         MOVEL_ACK_TIMEOUT_SEC)
from datetime import datetime

class RecoverableCommError(Exception):
    pass

DELIM_HASH = b'#'
DELIMS = (b'#', b'\n', b'\r')  # ← 区切り拡張

def _quat_wxyz_from_rpy(rpy_deg):
    """rpy(deg) -> (qw,qx,qy,qz) に並べ替え。ついでに正規化。"""
    qx, qy, qz, qw = rpy_deg_to_quat(*rpy_deg)  # ここは [x,y,z,w]
    n = (qx*qx + qy*qy + qz*qz + qw*qw) ** 0.5
    if n > 0:
        qx, qy, qz, qw = qx/n, qy/n, qz/n, qw/n
    return (qw, qx, qy, qz)


class RobotClient:
    def __init__(self, host, port):
        self._last_speed = None   # (v_tcp, v_ori)
        self._last_zone  = None   # (p_tcp, p_ori, z_ori)
        
        self.host = host; self.port = port
        self.sock: socket.socket | None = None
        self._sock_lock = threading.Lock()

        # 受信バッファ＆スレッド
        self._rxbuf = bytearray()
        self._alive = False
        self._reader_th: threading.Thread | None = None

        # CASEごとのメールボックス
        self._mbox = collections.defaultdict(collections.deque)

        # 旧API互換のダミー
        self._drain_stop = threading.Event()
        self._drain_thread: threading.Thread | None = None

        # アイドル判定用
        self._last_rx_ts = time.time()

    

    def set_speed_if_changed(self, v_tcp, v_ori):
        cur = (float(v_tcp), float(v_ori))
        if self._last_speed != cur:
            self.set_speed(v_tcp, v_ori)
            self._last_speed = cur

    def set_zone_if_changed(self, p_tcp, p_ori, z_ori):
        cur = (float(p_tcp), float(p_ori), float(z_ori))
        if self._last_zone != cur:
            self.set_zone(p_tcp, p_ori, z_ori)
            self._last_zone = cur

    # --- connect/close ---
    def connect(self, timeout=5.0):
        self.sock = socket.create_connection((self.host, self.port), timeout=timeout)
        self.sock.settimeout(DEFAULT_RECV_TIMEOUT)
        try: self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        except Exception: pass
        try: self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
        except Exception: pass

        self._alive = True
        self._reader_th = threading.Thread(target=self._reader_loop, daemon=True)
        self._reader_th.start()

        # 接続直後の安定待ち（初回ACK落ち防止）
        time.sleep(0.10)

        print(f"[INFO] Connected {self.host}:{self.port}")

    def close(self):
        try: self.stop_ack_drain()
        except: pass
        self._alive = False
        if self._reader_th:
            self._reader_th.join(timeout=0.5)
            self._reader_th = None
        try:
            if self.sock:
                self.sock.close()
        finally:
            self.sock = None

    # --- reader loop: 常時受信→フレーム化→CASE別メールボックスへ配送 ---
    def _emit_frame(self, frame_bytes: bytes):
        frame = frame_bytes.decode("ascii", errors="ignore").strip()
        if not frame:
            return
        toks = frame.split()
        case_id = toks[0] if toks else "?"
        self._mbox[case_id].append(frame)

    def _reader_loop(self):
        """常時受信し、'#'・LF・CR で分割。区切りが無ければアイドル時間で確定。"""
        idle_window = max(0.01, float(IDLE_FRAME_WINDOW_SEC))
        while self._alive and self.sock:
            try:
                r, _, _ = select.select([self.sock], [], [], 0.1)
                now = time.time()
                if r:
                    with self._sock_lock:
                        chunk = self.sock.recv(4096)
                    if not chunk:
                        self._alive = False
                        break
                    self._last_rx_ts = now
                    self._rxbuf.extend(chunk)

                    # 優先して明示区切りで切る
                    # 可能な限り多くのフレームを取り出す
                    while True:
                        # 最も手前の区切り位置を探す
                        cut_idx = -1
                        cut_len = 0
                        for d in DELIMS:
                            idx = self._rxbuf.find(d)
                            if idx != -1 and (cut_idx == -1 or idx < cut_idx):
                                cut_idx = idx
                                cut_len = len(d)
                        if cut_idx == -1:
                            break  # 区切りが無いので次回へ
                        frame_bytes = bytes(self._rxbuf[:cut_idx])
                        del self._rxbuf[:cut_idx + cut_len]
                        if frame_bytes:
                            self._emit_frame(frame_bytes)

                else:
                    # 受信が無い状態が続き、かつrxbufに溜まっていたら「アイドル切り」
                    if self._rxbuf and (now - self._last_rx_ts) >= idle_window:
                        frame_bytes = bytes(self._rxbuf)
                        self._rxbuf.clear()
                        self._emit_frame(frame_bytes)

            except (BlockingIOError, socket.timeout):
                continue
            except Exception:
                time.sleep(0.02)
                continue

    # --- 旧API互換（実質NOP） ---
    def start_ack_drain(self): return
    def _drain_loop(self): 
        while False: time.sleep(0.1)
    def stop_ack_drain(self):
        return  # 完全NOP。スリープとprintをやめる


    def flush_recv(self):
        self._rxbuf = bytearray()
        # 必要ならメールボックスもクリア:
        # for q in self._mbox.values(): q.clear()

    # --- 共通 send/recv ---
    def _send_and_recv(self, parts, recv_timeout=None, expect=None,
                       pause=0.02, allow_stray_move_ack=False):
        if not self.sock: raise RuntimeError("Socket not connected")
        msg = " ".join(fmt(p) for p in parts) + " #"
        with self._sock_lock:
            self.sock.sendall(msg.encode("ascii"))
        if pause > 0:
            time.sleep(pause)

        deadline = time.time() + (recv_timeout if recv_timeout else DEFAULT_RECV_TIMEOUT)
        expect_key = str(expect) if expect is not None else None
        last_raw = ""

        while time.time() < deadline:
            # 期待CASEが来ていれば即返す
            if expect_key is not None and self._mbox[expect_key]:
                raw = self._mbox[expect_key].popleft()
                last_raw = raw
                ts = datetime.now().strftime('%H:%M:%S.%f')[:-3]  # ミリ秒まで
                print(f"{ts} [SEND] {msg.strip()}   [RECV] {raw}")
                toks = raw.split()
                instr = int(toks[0]) if len(toks) >= 1 and toks[0].lstrip("-").isdigit() else None
                ok    = int(toks[1]) if len(toks) >= 2 and toks[1].lstrip("-").isdigit() else None
                if instr is None or ok is None:
                    continue
                return instr, ok, toks

            # MoveLの迷子ACK救済
            if allow_stray_move_ack and self._mbox["1"]:
                raw = self._mbox["1"].popleft()
                last_raw = raw
                ts = datetime.now().strftime('%H:%M:%S.%f')[:-3]  # ミリ秒まで
                print(f"{ts} [SEND] {msg.strip()}   [RECV] {raw}")
                toks = raw.split()
                instr = int(toks[0]) if len(toks) >= 1 and toks[0].lstrip("-").isdigit() else None
                ok    = int(toks[1]) if len(toks) >= 2 and toks[1].lstrip("-").isdigit() else None
                if instr is None or ok is None:
                    continue
                if expect is None or instr == expect:
                    return instr, ok, toks

            time.sleep(0.005)

        # タイムアウト時に中身を可視化
        mbox_snapshot = {k: len(v) for k, v in self._mbox.items() if len(v) > 0}
        raise RecoverableCommError(
            f"Timeout waiting ACK for CASE {expect}: "
            f"sent='{msg.strip()}', last_raw='{last_raw}', mbox={mbox_snapshot}"
        )

    # --- wrappers (open_abb CASEs) ---
    def ping(self, tries=3):
        last = None
        for i in range(tries):
            try:
                return self._send_and_recv([0], expect=0, recv_timeout=2.5, pause=0.05, allow_stray_move_ack=True)
            except Exception as e:
                last = e
                time.sleep(0.2 + 0.2*i)
        raise last

    def set_tool(self, txyz, rpy_deg):
        qw, qx, qy, qz = _quat_wxyz_from_rpy(rpy_deg)
        return self._send_and_recv([6, *txyz, qw, qx, qy, qz], expect=6)


    def set_wobj(self, identity=True, txyz=(0,0,0), rpy_deg=(0,0,0)):
        if identity:
            return self._send_and_recv([7, 0.0,0.0,0.0, 1.0,0.0,0.0,0.0], expect=7)
        else:
            qw, qx, qy, qz = _quat_wxyz_from_rpy(rpy_deg)
            return self._send_and_recv([7, *txyz, qw, qx, qy, qz], expect=7)


    def set_speed(self, v_tcp, v_ori):
        return self._send_and_recv([8, v_tcp, v_ori], expect=8, allow_stray_move_ack=True)

    def set_zone(self, p_tcp, p_ori, z_ori):
        return self._send_and_recv([9, 0, p_tcp, p_ori, z_ori], expect=9, allow_stray_move_ack=True)



    def get_cart(self):
        instr, ok, toks = self._send_and_recv([3], expect=3)
        if ok != 1 or len(toks) < 9:
            raise RuntimeError(f"CASE3 reply invalid: '{' '.join(toks)}'")
        x = float(toks[2]); y = float(toks[3]); z = float(toks[4])
        # サーバは [qw,qx,qy,qz] を返す前提でパース
        qw = float(toks[5]); qx = float(toks[6]); qy = float(toks[7]); qz = float(toks[8])
        return (x, y, z, qx, qy, qz, qw)


    def moveL_ack(self, x, y, z, rpy_deg, ack_timeout=MOVEL_ACK_TIMEOUT_SEC):
        qw, qx, qy, qz = _quat_wxyz_from_rpy(rpy_deg)
        return self._send_and_recv([1, x, y, z, qw, qx, qy, qz],
                                recv_timeout=ack_timeout, expect=1)


    def moveL_send_only(self, x, y, z, rpy_deg, tiny_pause=0.03):
        if not self.sock: raise RuntimeError("Socket not connected")
        qw, qx, qy, qz = _quat_wxyz_from_rpy(rpy_deg)
        msg = " ".join(fmt(p) for p in [1, x, y, z, qw, qx, qy, qz]) + " #"
        with self._sock_lock:
            self.sock.sendall(msg.encode("ascii"))
        if tiny_pause > 0:
            time.sleep(tiny_pause)

    
    
