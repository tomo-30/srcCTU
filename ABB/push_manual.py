# -*- coding: utf-8 -*-
# plan_runner_hybrid_framing.py
# ・push_plan.json を A1,B1,A1,A2,B2,A2,... の順で実行
# ・受信は「# 終端」**または**「アイドル（無通信）」で1フレーム確定
# ・遅延MoveL ACK(instr=1)は期待外でも読み飛ばす
# ・ACKドレイン＋停止後小休止＋送信スロットルで詰まり緩和

import os
import json
import socket
import select
import time
import math
import threading
import tkinter as tk
from tkinter import ttk, messagebox

# ===== 接続先 =====
ROBOT_IP, ROBOT_PORT = "192.168.125.1", 5000

# ===== 既定パラメータ =====
TOOL_TXYZ = (0.0, 0.0, 0.0)
TOOL_RPY  = (0.0, 0.0, 0.0)
IDENTITY_WOBJ = True
WOBJ_TXYZ = (0.0, 0.0, 0.0)
WOBJ_RPY  = (0.0, 0.0, 0.0)

DEFAULT_STEP_MM = 4.0
DEFAULT_V_TCP   = 40
DEFAULT_V_ORI   = 80
DEFAULT_P_TCP   = 1.5
DEFAULT_P_ORI   = 8
DEFAULT_Z_ORI   = 8
DEFAULT_LOOKAHEAD = 3
DEFAULT_FINAL_FINE = True
DEFAULT_FINAL_P_TCP = 0.5

MOVEL_ACK_TIMEOUT_SEC = 60.0
DEFAULT_RECV_TIMEOUT   = 3.0         # ブロッキング時の基準
IDLE_FRAME_WINDOW_SEC  = 0.07        # 受信アイドルでフレーム確定する閾値

PLAN_FILENAME = "push_plan.json"

class RecoverableCommError(Exception):
    """通信の一時的な乱れ。再開で継続できる類のエラー。"""
    pass

def fmt(n):
    return str(n) if isinstance(n, int) else f"{float(n):.6f}"

def rpy_deg_to_quat(r, p, y):
    rx, ry, rz = map(math.radians, (r, p, y))
    cx, sx = math.cos(rx/2), math.sin(rx/2)
    cy, sy = math.cos(ry/2), math.sin(ry/2)
    cz, sz = math.cos(rz/2), math.sin(rz/2)
    qw = cz*cy*cx + sz*sy*sx
    qx = cz*cy*sx - sz*sy*cx
    qy = cz*sy*cx + sz*cy*sx
    qz = sz*cy*cx - cz*sy*sx
    n = math.sqrt(qw*qw + qx*qx + qy*qy + qz*qz) or 1.0
    return [qx/n, qy/n, qz/n, qw/n]

def make_via_points_by_step(curr_xyz, target_xyz, step_mm: float):
    cx, cy, cz = curr_xyz
    tx, ty, tz = target_xyz
    dx, dy, dz = tx - cx, ty - cy, tz - cz
    dist = math.sqrt(dx*dx + dy*dy + dz*dz)
    if dist <= 1e-6:
        return [target_xyz]
    n_seg = max(1, int(dist // max(step_mm, 1e-6)))
    via = []
    for i in range(1, n_seg + 1):
        s = i / n_seg
        via.append((cx + dx*s, cy + dy*s, cz + dz*s))
    via[-1] = (tx, ty, tz)
    return via

# ===== RobotClient =====
class RobotClient:
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.sock: socket.socket | None = None
        self._sock_lock = threading.Lock()
        self._rxbuf = b""
        # ドレイン
        self._drain_stop = threading.Event()
        self._drain_thread: threading.Thread | None = None

    def connect(self, timeout=5.0):
        self.sock = socket.create_connection((self.host, self.port), timeout=timeout)
        # Pythonソケットのタイムアウトは参考用，実受信はselectで制御
        self.sock.settimeout(DEFAULT_RECV_TIMEOUT)
        print(f"[INFO] Connected {self.host}:{self.port}")

    def close(self):
        try:
            self.stop_ack_drain()
        except:
            pass
        try:
            if self.sock:
                self.sock.close()
        finally:
            self.sock = None

    # ---- ハイブリッド受信: '#' か アイドルで1フレーム確定 ----
    def _recv_frame(self, deadline: float) -> str:
        """
        1) _rxbuf に '#' があればそこまでを返す
        2) なければ select で IDLE_FRAME_WINDOW_SEC 待つ
           - その間にデータが来れば追記して1)へ
           - 来なければ，_rxbuf 全体を1フレームとして返す（'#'無し対応）
        3) deadline 超過で Timeout（ただし _rxbuf が非空ならそれを返す）
        """
        while True:
            # 1) '#' でのフレーム確定
            idx = self._rxbuf.find(b'#')
            if idx != -1:
                frame = self._rxbuf[:idx].decode("ascii", errors="ignore").strip()
                self._rxbuf = self._rxbuf[idx+1:]
                if frame:
                    return frame

            # 期限超過なら残りを返すかタイムアウト
            now = time.time()
            if now > deadline:
                if self._rxbuf:
                    frame = self._rxbuf.decode("ascii", errors="ignore").strip()
                    self._rxbuf = b""
                    if frame:
                        return frame
                raise TimeoutError("recv timeout while waiting for '#'")

            # 2) 少しだけ受信待ち
            wait = min(IDLE_FRAME_WINDOW_SEC, max(0.0, deadline - now))
            r, _, _ = select.select([self.sock], [], [], wait)
            if r:
                with self._sock_lock:
                    chunk = self.sock.recv(4096)
                if chunk:
                    self._rxbuf += chunk
                    continue
                else:
                    # ソケットは準備OKだったが0byte→一旦アイドル扱い
                    pass
            # 待機中に新規データが無かった→現在のバッファを1フレームとして扱う
            if self._rxbuf:
                frame = self._rxbuf.decode("ascii", errors="ignore").strip()
                self._rxbuf = b""
                if frame:
                    return frame
            # まだ何もない→ループ継続

    # ---- ドレイン ----
    def start_ack_drain(self):
        if not self.sock:
            raise RuntimeError("Socket not connected")
        if self._drain_thread and self._drain_thread.is_alive():
            return
        with self._sock_lock:
            self.sock.setblocking(False)
        self._drain_stop.clear()
        self._drain_thread = threading.Thread(target=self._drain_loop, daemon=True)
        self._drain_thread.start()
        print("[INFO] ACK drain started.")

    def _drain_loop(self):
        while not self._drain_stop.is_set():
            try:
                with self._sock_lock:
                    if not self.sock:
                        break
                    _ = self.sock.recv(4096)
                time.sleep(0.002)
            except (BlockingIOError, socket.timeout):
                time.sleep(0.004)
            except Exception:
                time.sleep(0.02)

    def stop_ack_drain(self):
        if not self._drain_thread:
            return
        self._drain_stop.set()
        self._drain_thread.join(timeout=0.5)
        self._drain_thread = None
        with self._sock_lock:
            if self.sock:
                self.sock.setblocking(True)
                self.sock.settimeout(DEFAULT_RECV_TIMEOUT)
        time.sleep(0.03)  # レース緩和
        print("[INFO] ACK drain stopped.")

    def flush_recv(self):
        if not self.sock:
            return
        self._rxbuf = b""
        with self._sock_lock:
            old_to = self.sock.gettimeout()
            self.sock.setblocking(False)
            total = 0
            try:
                while True:
                    try:
                        chunk = self.sock.recv(4096)
                        if not chunk:
                            break
                        total += len(chunk)
                    except (BlockingIOError, socket.timeout):
                        break
            finally:
                self.sock.setblocking(True)
                self.sock.settimeout(old_to)
        if total:
            print(f"[INFO] Flushed {total} bytes of leftover ACKs.")

    # ---- 送受信共通（遅延MoveL ACK読み飛ばし付き） ----
    def _send_and_recv(self, parts, recv_timeout=None, expect=None,
                   pause=0.12, allow_stray_move_ack=False):
        if not self.sock:
            raise RuntimeError("Socket not connected")
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
                # ← ここを“回復可能”にする
                raise RecoverableCommError(str(te)) from te

            last_raw = raw
            print(f"[SEND] {msg.strip()}   [RECV] {raw}")
            toks = raw.split()
            instr = int(toks[0]) if len(toks) >= 1 and toks[0].lstrip("-").isdigit() else None
            ok    = int(toks[1]) if len(toks) >= 2 and toks[1].lstrip("-").isdigit() else None

            if instr is None or ok is None:
                # ゴミは読み飛ばし（次のフレームを待つ）
                continue

            if (expect is None) or (instr == expect):
                return instr, ok, toks

            if allow_stray_move_ack and instr == 1:
                # 期待外の MoveL ACK は読み飛ばして続行
                continue

            # ← ここも“回復可能”にする
            raise RecoverableCommError(
                f"ACK instr mismatch: expected {expect}, got {instr}, raw='{last_raw}'"
            )


    # ---- open_abb wrappers ----
    def ping(self):
        return self._send_and_recv([0], expect=0, allow_stray_move_ack=True)

    def set_tool(self, txyz, rpy_deg):
        tq = rpy_deg_to_quat(*rpy_deg)
        return self._send_and_recv([6, *txyz, *tq], expect=6, allow_stray_move_ack=True)

    def set_wobj(self, identity=True, txyz=(0,0,0), rpy_deg=(0,0,0)):
        if identity:
            return self._send_and_recv([7, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
                                       expect=7, allow_stray_move_ack=True)
        else:
            wq = rpy_deg_to_quat(*rpy_deg)
            return self._send_and_recv([7, *txyz, *wq], expect=7, allow_stray_move_ack=True)

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
        if not self.sock:
            raise RuntimeError("Socket not connected")
        qx, qy, qz, qw = rpy_deg_to_quat(*rpy_deg)
        msg = " ".join(fmt(p) for p in [1, x, y, z, qx, qy, qz, qw]) + " #"
        with self._sock_lock:
            self.sock.sendall(msg.encode("ascii"))
        time.sleep(tiny_pause)

# ===== push_plan 読み込み =====
def load_plan_sequence(json_path):
    with open(json_path, "r", encoding="utf-8") as f:
        data = json.load(f)
    plan = data.get("plan", [])
    plan_sorted = sorted(plan, key=lambda d: d.get("id", 0))
    seq = []
    for item in plan_sorted:
        _id = item.get("id")
        A = item.get("A"); B = item.get("B")
        if not (A and B) or len(A) < 6 or len(B) < 6:
            raise ValueError(f"id={_id} のA/Bが不正（[x,y,z,roll,pitch,yaw]の6要素）")
        Ax,Ay,Az,Ar,Ap,Ayaw = map(float, A)
        Bx,By,Bz,Br,Bp,Byaw = map(float, B)
        seq.append((f"A{_id}", (Ax,Ay,Az,Ar,Ap,Ayaw)))
        seq.append((f"B{_id}", (Bx,By,Bz,Br,Bp,Byaw)))
        seq.append((f"A{_id}", (Ax,Ay,Az,Ar,Ap,Ayaw)))
    return seq

# ===== GUI =====
class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("open_abb push_plan runner (framed/idle I/O + ACK drain)")
        self.geometry("780x540")

        self.client = RobotClient(ROBOT_IP, ROBOT_PORT)
        self.pause_flag = threading.Event()
        self.skip_flag  = threading.Event()
        self.motion_thread: threading.Thread | None = None

        self.sequence = []
        self.seq_idx = 0

        top = ttk.LabelFrame(self, text="現在の目的地"); top.pack(padx=10, pady=8, fill="x")
        self.var_curr_label = tk.StringVar(value="-")
        self.var_curr_xyzrpy = tk.StringVar(value="x= -, y= -, z= -, roll= -, pitch= -, yaw= -")
        ttk.Label(top, textvariable=self.var_curr_label, font=("", 14, "bold"), width=10)\
            .grid(row=0, column=0, sticky="w", padx=6, pady=6)
        ttk.Label(top, textvariable=self.var_curr_xyzrpy, font=("", 12))\
            .grid(row=0, column=1, sticky="w", padx=6, pady=6)

        seg = ttk.LabelFrame(self, text="パラメータ"); seg.pack(padx=10, pady=6, fill="x")
        self.var_step_mm   = tk.StringVar(value=str(DEFAULT_STEP_MM))
        self.var_lookahead = tk.StringVar(value=str(DEFAULT_LOOKAHEAD))
        self.var_v_tcp     = tk.StringVar(value=str(DEFAULT_V_TCP))
        self.var_v_ori     = tk.StringVar(value=str(DEFAULT_V_ORI))
        self.var_p_tcp     = tk.StringVar(value=str(DEFAULT_P_TCP))
        self.var_p_ori     = tk.StringVar(value=str(DEFAULT_P_ORI))
        self.var_z_ori     = tk.StringVar(value=str(DEFAULT_Z_ORI))
        self.var_final_p   = tk.StringVar(value=str(DEFAULT_FINAL_P_TCP))
        self.var_final_fine= tk.BooleanVar(value=DEFAULT_FINAL_FINE)

        grid = [
            ("ステップ長 step(mm):", self.var_step_mm),
            ("lookahead:", self.var_lookahead),
            ("V_TCP(mm/s):", self.var_v_tcp),
            ("V_ORI(deg/s):", self.var_v_ori),
            ("P_TCP(mm):", self.var_p_tcp),
            ("P_ORI(deg):", self.var_p_ori),
            ("Z_ORI(deg):", self.var_z_ori),
            ("終点 P_TCP(mm):", self.var_final_p),
        ]
        for i,(lab,var) in enumerate(grid):
            ttk.Label(seg, text=lab).grid(row=i//4, column=(i%4)*2, sticky="e", padx=4, pady=4)
            ttk.Entry(seg, textvariable=var, width=8).grid(row=i//4, column=(i%4)*2+1, sticky="w")
        ttk.Checkbutton(seg, text="終点のみFine収束（P_TCPを一時的に小さく）",
                        variable=self.var_final_fine)\
            .grid(row=2, column=0, columnspan=4, sticky="w", padx=6, pady=4)

        btns = ttk.Frame(self); btns.pack(padx=10, pady=10, fill="x")
        ttk.Button(btns, text="開始", command=self.on_start).pack(side="left", padx=6)
        self.btn_pause = ttk.Button(btns, text="一時停止", command=self.on_pause_resume)
        self.btn_pause.pack(side="left", padx=6)
        ttk.Button(btns, text="次へ（スキップ）", command=self.on_skip).pack(side="left", padx=6)
        ttk.Button(btns, text="計画を再読込", command=self.on_reload_plan).pack(side="left", padx=6)

        self.after(100, self._auto_connect_and_init)

    def _auto_connect_and_init(self):
        try:
            print("[INFO] Connecting...")
            self.client.connect(timeout=5.0)
            self.client.ping()
            self.client.set_wobj(identity=IDENTITY_WOBJ, txyz=WOBJ_TXYZ, rpy_deg=WOBJ_RPY)
            self.client.set_tool(TOOL_TXYZ, TOOL_RPY)
            self._apply_speed_zone_from_gui()
            self._load_plan_from_file()
            messagebox.showinfo("接続", "ロボットに接続し，計画を読み込みました。")
        except Exception as e:
            messagebox.showerror("接続エラー", f"{e}")

    def _apply_speed_zone_from_gui(self):
        v_tcp = float(self.var_v_tcp.get())
        v_ori = float(self.var_v_ori.get())
        p_tcp = float(self.var_p_tcp.get())
        p_ori = float(self.var_p_ori.get())
        z_ori = float(self.var_z_ori.get())
        self.client.set_speed(v_tcp, v_ori)
        self.client.set_zone(p_tcp, p_ori, z_ori)

    def _load_plan_from_file(self):
        base = os.path.dirname(os.path.abspath(__file__))
        path = os.path.join(base, PLAN_FILENAME)
        if not os.path.exists(path):
            raise FileNotFoundError(f"{PLAN_FILENAME} が見つかりません: {path}")
        self.sequence = load_plan_sequence(path)
        self.seq_idx = 0
        self._update_current_target_label()

    def on_start(self):
        if self.motion_thread and self.motion_thread.is_alive():
            messagebox.showwarning("実行中", "既に移動中です。一時停止やスキップを使用してください。")
            return
        try:
            self.client.flush_recv()
            self._apply_speed_zone_from_gui()
        except Exception as e:
            messagebox.showerror("入力エラー", f"{e}")
            return
        self.pause_flag.clear()
        self.skip_flag.clear()
        self.motion_thread = threading.Thread(target=self._run_plan_loop, daemon=True)
        self.motion_thread.start()

    def on_pause_resume(self):
        if not (self.motion_thread and self.motion_thread.is_alive()):
            return
        if not self.pause_flag.is_set():
            self.pause_flag.set()
            self.btn_pause.config(text="再開")
        else:
            self.client.flush_recv()
            self.pause_flag.clear()
            self.btn_pause.config(text="一時停止")

    def on_skip(self):
        if self.motion_thread and self.motion_thread.is_alive():
            self.skip_flag.set()

    def on_reload_plan(self):
        try:
            self._load_plan_from_file()
            messagebox.showinfo("読込", "push_plan.json を再読み込みしました。")
        except Exception as e:
            messagebox.showerror("読込エラー", f"{e}")

    def _update_current_target_label(self):
        if 0 <= self.seq_idx < len(self.sequence):
            label, (x,y,z,r,p,yaw) = self.sequence[self.seq_idx]
            self.var_curr_label.set(label)
            self.var_curr_xyzrpy.set(
                f"x={x:.1f}, y={y:.1f}, z={z:.1f}, roll={r:.1f}, pitch={p:.1f}, yaw={yaw:.1f}"
            )
        else:
            self.var_curr_label.set("-")
            self.var_curr_xyzrpy.set("x= -, y= -, z= -, roll= -, pitch= -, yaw= -")

    def _run_plan_loop(self):
        try:
            while self.seq_idx < len(self.sequence):
                while self.pause_flag.is_set():
                    time.sleep(0.05)

                label, pose = self.sequence[self.seq_idx]
                self._update_current_target_label()

                ok = self._run_single_target(pose)
                if ok is None:
                    self.client.flush_recv()
                    self.skip_flag.clear()
                    self.seq_idx += 1
                    self._update_current_target_label()
                    continue
                elif ok is False:
                    break
                else:
                    self.seq_idx += 1
                    self._update_current_target_label()

            print("[INFO] Plan loop finished.")
        except Exception as e:
            try:
                self.client.stop_ack_drain()
                self.client.flush_recv()
            except:
                pass
            messagebox.showerror("実行エラー", f"{e}")

    def _run_single_target(self, target_xyzrpy):
        while True:  # ← ここで“同じ目標”をリトライ可能にする
            try:
                self.client.flush_recv()
                cx, cy, cz, *_ = self.client.get_cart()

                step_mm   = float(self.var_step_mm.get());   assert step_mm > 0
                lookahead = int(float(self.var_lookahead.get())); assert lookahead >= 1
                final_fine = bool(self.var_final_fine.get())
                final_p_tcp= float(self.var_final_p.get())
                if final_fine and final_p_tcp <= 0:
                    raise ValueError("終点P_TCPは正の値にしてください。")

                tx, ty, tz, rr, pp, yw = target_xyzrpy
                via_xyz = make_via_points_by_step((cx,cy,cz), (tx,ty,tz), step_mm=step_mm)
                n = len(via_xyz)

                self.client.start_ack_drain()

                if n == 1:
                    self.client.stop_ack_drain()
                    self._move_final_with_optional_fine(via_xyz[-1], (rr,pp,yw), final_fine, final_p_tcp)
                    return True

                preload = min(lookahead, max(0, n-1))
                idx_sent = -1
                for k in range(preload):
                    if self._check_pause_skip():
                        self.client.stop_ack_drain(); self.client.flush_recv()
                        return None if self.skip_flag.is_set() else False
                    vx, vy, vz = via_xyz[k]
                    print(f"[PRELOAD] {k+1}/{n-1} -> ({vx:.2f},{vy:.2f},{vz:.2f}) send_only")
                    self.client.moveL_send_only(vx, vy, vz, (rr,pp,yw))
                    idx_sent = k

                while idx_sent < n-2:
                    if self._check_pause_skip():
                        self.client.stop_ack_drain(); self.client.flush_recv()
                        return None if self.skip_flag.is_set() else False
                    idx_sent += 1
                    nxt = idx_sent
                    if nxt < n-1:
                        vx, vy, vz = via_xyz[nxt]
                        print(f"[PIPE] {nxt+1}/{n-1} -> ({vx:.2f},{vy:.2f},{vz:.2f}) send_only")
                        self.client.moveL_send_only(vx, vy, vz, (rr,pp,yw))
                    time.sleep(0.04)

                self.client.stop_ack_drain()
                self.client.flush_recv()
                if self._check_pause_skip():
                    return None if self.skip_flag.is_set() else False

                self._move_final_with_optional_fine(via_xyz[-1], (rr,pp,yw), final_fine, final_p_tcp)
                return True

            except RecoverableCommError as e:
                # ★ここがキモ：移動を“中断”に留める
                print(f"[WARN] recoverable comm error -> auto-pause: {e}")
                try:
                    self.client.stop_ack_drain()
                    self.client.flush_recv()
                except:
                    pass

                # 自動で一時停止に切り替え（GUI表示も一致させる）
                if not self.pause_flag.is_set():
                    self.pause_flag.set()
                    self.btn_pause.config(text="再開")

                # 再開 or スキップを待つ（停止ボタンのレスポンスには影響させない）
                while self.pause_flag.is_set() and not self.skip_flag.is_set():
                    time.sleep(0.05)

                if self.skip_flag.is_set():
                    self.skip_flag.clear()
                    return None  # “次へ”扱い

                # ここまで来たら“再開”→ ループ先頭に戻って
                # 現在姿勢を読み直し、同じターゲットを再実行
                continue

            except Exception:
                # それ以外は従来通り“致命的”に扱う
                try:
                    self.client.stop_ack_drain()
                    self.client.flush_recv()
                except:
                    pass
                raise


    def _check_pause_skip(self):
        while self.pause_flag.is_set():
            time.sleep(0.02)
        return self.skip_flag.is_set()

    def _move_final_with_optional_fine(self, final_xyz, target_rpy, final_fine, final_p_tcp):
        if final_fine:
            p_tcp = float(self.var_p_tcp.get())
            p_ori = float(self.var_p_ori.get())
            z_ori = float(self.var_z_ori.get())
            try:
                self.client.set_zone(final_p_tcp, p_ori, z_ori)
                time.sleep(0.02)
                self.client.moveL_ack(final_xyz[0], final_xyz[1], final_xyz[2], target_rpy,
                                      ack_timeout=MOVEL_ACK_TIMEOUT_SEC)
            finally:
                time.sleep(0.02)
                self.client.set_zone(p_tcp, p_ori, z_ori)
        else:
            self.client.moveL_ack(final_xyz[0], final_xyz[1], final_xyz[2], target_rpy,
                                  ack_timeout=MOVEL_ACK_TIMEOUT_SEC)

    def destroy(self):
        try:
            self.client.close()
        except:
            pass
        super().destroy()

if __name__ == "__main__":
    app = App()
    app.mainloop()
