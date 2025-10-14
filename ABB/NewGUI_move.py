# -*- coding: utf-8 -*-
# NewGUI_move.py
# 目的:
#   ・中継点(分割)を連続ブレンドさせるため、中間点はACK待ちなしで先行送信（lookahead）
#   ・サーバACK未読による詰まりを防ぐため、ACKドレイン（読み捨て）スレッドを実装
#   ・停止→再開時の未読ACK残りを flush で確実に掃除
#   ・最後の1点のみACK待ち(60s)で確実に完了
#   ・停止ボタンは「以降の送信停止」で安全に減速停止
#
# 前提: open_abb SERVER（CASE 0/1/3/6/7/8/9対応）が動作

import socket
import time
import math
import threading
import tkinter as tk
from tkinter import ttk, messagebox

# ====== 接続先 ======
ROBOT_IP, ROBOT_PORT = "192.168.125.1", 5000

# ====== 既定パラメータ ======
# Tool / WObj は単位姿勢（ツール先＝フランジ，WObj＝ベース）
TOOL_TXYZ = (0.0, 0.0, 0.0)
TOOL_RPY  = (0.0, 0.0, 0.0)
IDENTITY_WOBJ = True
WOBJ_TXYZ = (0.0, 0.0, 0.0)
WOBJ_RPY  = (0.0, 0.0, 0.0)

# 初期値（中継点 1 mm 前提のおすすめ）
DEFAULT_STEP_MM = 1.0
DEFAULT_V_TCP   = 40    # mm/s
DEFAULT_V_ORI   = 80    # deg/s
DEFAULT_P_TCP   = 1.5   # mm
DEFAULT_P_ORI   = 8     # deg
DEFAULT_Z_ORI   = 8     # deg
DEFAULT_LOOKAHEAD = 3   # 2〜4推奨
DEFAULT_FINAL_FINE = True
DEFAULT_FINAL_P_TCP = 0.5  # mm（終点だけピタ止めしたい場合）

# MoveL のみACK待ちを長めに（A案）
MOVEL_ACK_TIMEOUT_SEC = 60.0
# ソケット共通の短いタイムアウト（PingやGet、Set系）
DEFAULT_RECV_TIMEOUT = 3.0

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
    return [qx/n, qy/n, qz/n, qw/n]  # ABB順: qx,qy,qz,qw

class RobotClient:
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.sock: socket.socket | None = None
        # 送受信排他
        self._sock_lock = threading.Lock()
        # ACKドレイン
        self._drain_stop = threading.Event()
        self._drain_thread: threading.Thread | None = None

    # ---- 接続管理 ----
    def connect(self, timeout=5.0):
        self.sock = socket.create_connection((self.host, self.port), timeout=timeout)
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

    # ---- ACKドレイン（読み捨て） ----
    def start_ack_drain(self):
        if not self.sock:
            raise RuntimeError("Socket not connected")
        if self._drain_thread and self._drain_thread.is_alive():
            return
        with self._sock_lock:
            self.sock.setblocking(False)  # 非ブロッキング受信へ
        self._drain_stop.clear()
        self._drain_thread = threading.Thread(target=self._drain_loop, daemon=True)
        self._drain_thread.start()
        print("[INFO] ACK drain started.")

    def stop_ack_drain(self):
        if not self._drain_thread:
            return
        self._drain_stop.set()
        self._drain_thread.join(timeout=0.5)
        self._drain_thread = None
        # 既定（ブロッキング＋タイムアウト）へ復帰
        with self._sock_lock:
            if self.sock:
                self.sock.setblocking(True)
                self.sock.settimeout(DEFAULT_RECV_TIMEOUT)
        print("[INFO] ACK drain stopped.")

    def _drain_loop(self):
        # サーバからのACKを読み捨て続ける
        while not self._drain_stop.is_set():
            try:
                with self._sock_lock:
                    if not self.sock:
                        break
                    data = self.sock.recv(4096)
                if not data:
                    time.sleep(0.01)
                # 必要ならログ: print("[DRAIN]", data.decode(errors="ignore"))
            except BlockingIOError:
                time.sleep(0.004)   # データなし
            except Exception:
                time.sleep(0.02)    # 一時的な例外はスルーして継続

    # ---- 受信フラッシュ（残りACK掃除） ----
    def flush_recv(self):
        """非ブロッキングで受信バッファを空にする（停止→再開前などに呼ぶ）"""
        if not self.sock:
            return
        with self._sock_lock:
            old_block = self.sock.gettimeout()  # ブロッキング状態を後で復元
            self.sock.setblocking(False)
            try:
                total = 0
                while True:
                    try:
                        chunk = self.sock.recv(4096)
                        if not chunk:
                            break
                        total += len(chunk)
                    except BlockingIOError:
                        break
            finally:
                self.sock.setblocking(True)
                self.sock.settimeout(DEFAULT_RECV_TIMEOUT)
        if total:
            print(f"[INFO] Flushed {total} bytes of leftover ACKs.")

    # ---- 共通送受信 ----
    def _send_and_recv(self, parts, recv_timeout=None, expect=None, pause=0.12):
        if not self.sock:
            raise RuntimeError("Socket not connected")
        msg = " ".join(fmt(p) for p in parts) + " #"
        with self._sock_lock:
            self.sock.sendall(msg.encode("ascii"))
        time.sleep(pause)  # サーバの受理間隔を少し確保

        # ここはブロッキングで受ける（ドレイン停止中であることが前提）
        old_to = None
        if recv_timeout is not None:
            with self._sock_lock:
                old_to = self.sock.gettimeout()
                self.sock.settimeout(recv_timeout)
        try:
            with self._sock_lock:
                data = self.sock.recv(4096).decode("ascii", errors="ignore").strip()
        finally:
            if recv_timeout is not None:
                with self._sock_lock:
                    self.sock.settimeout(old_to)

        print(f"[SEND] {msg.strip()}   [RECV] {data}")
        toks = data.split()
        instr = int(toks[0]) if len(toks) >= 1 and toks[0].lstrip("-").isdigit() else None
        ok    = int(toks[1]) if len(toks) >= 2 and toks[1].lstrip("-").isdigit() else None
        if expect is not None and instr != expect:
            raise RuntimeError(f"ACK instr mismatch: expected {expect}, got {instr}, raw='{data}'")
        return instr, ok, toks

    # --- open_abb CASE wrappers ---
    def ping(self):
        return self._send_and_recv([0], expect=0)

    def set_tool(self, txyz, rpy_deg):
        tq = rpy_deg_to_quat(*rpy_deg)
        return self._send_and_recv([6, *txyz, *tq], expect=6)

    def set_wobj(self, identity=True, txyz=(0,0,0), rpy_deg=(0,0,0)):
        if identity:
            # 単位姿勢
            return self._send_and_recv([7, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0], expect=7)
        else:
            wq = rpy_deg_to_quat(*rpy_deg)
            return self._send_and_recv([7, *txyz, *wq], expect=7)

    def set_speed(self, v_tcp, v_ori):
        return self._send_and_recv([8, v_tcp, v_ori], expect=8)

    def set_zone(self, p_tcp, p_ori, z_ori):
        return self._send_and_recv([9, 0, p_tcp, p_ori, z_ori], expect=9)

    def get_cart(self):
        instr, ok, toks = self._send_and_recv([3], expect=3)
        if ok != 1 or len(toks) < 9:
            raise RuntimeError("CASE3 reply invalid")
        x = float(toks[2]); y = float(toks[3]); z = float(toks[4])
        qx = float(toks[5]); qy = float(toks[6]); qz = float(toks[7]); qw = float(toks[8])
        return (x, y, z, qx, qy, qz, qw)

    # --- MoveL（ACK待ち）---
    def moveL_ack(self, x, y, z, rpy_deg, ack_timeout=MOVEL_ACK_TIMEOUT_SEC):
        qx, qy, qz, qw = rpy_deg_to_quat(*rpy_deg)
        return self._send_and_recv([1, x, y, z, qx, qy, qz, qw],
                                   recv_timeout=ack_timeout, expect=1)

    # --- MoveL（送信のみ：ACK待たない）---
    def moveL_send_only(self, x, y, z, rpy_deg, tiny_pause=0.02):
        if not self.sock:
            raise RuntimeError("Socket not connected")
        qx, qy, qz, qw = rpy_deg_to_quat(*rpy_deg)
        msg = " ".join(fmt(p) for p in [1, x, y, z, qx, qy, qz, qw]) + " #"
        with self._sock_lock:
            self.sock.sendall(msg.encode("ascii"))
        time.sleep(tiny_pause)  # サーバの取りこぼし防止

# ---- 経由点生成（直線を step_mm で分割）----
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

# ============================================================
# GUI
# ============================================================

class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("open_abb MoveL (pipeline + ACK drain + flush)")
        self.geometry("680x440")

        self.client = RobotClient(ROBOT_IP, ROBOT_PORT)
        self.stop_flag = threading.Event()
        self.motion_thread: threading.Thread | None = None

        # --- 上段：目標入力 ---
        frm = ttk.Frame(self); frm.pack(padx=10, pady=8, fill="x")
        self.vars = {}
        labels = ["X(mm)","Y(mm)","Z(mm)","Roll(deg)","Pitch(deg)","Yaw(deg)"]
        defaults = [400.0, 0.0, 100.0, 0.0, 180.0, 0.0]
        for i,(lab,defv) in enumerate(zip(labels, defaults)):
            ttk.Label(frm, text=lab, width=12).grid(row=i//3, column=(i%3)*2, sticky="e", padx=4, pady=4)
            v = tk.StringVar(value=str(defv)); self.vars[lab]=v
            ttk.Entry(frm, textvariable=v, width=10).grid(row=i//3, column=(i%3)*2+1, sticky="w")

        # --- 中段：分割・速度・ゾーン・lookahead ---
        seg = ttk.LabelFrame(self, text="パラメータ"); seg.pack(padx=10, pady=6, fill="x")

        ttk.Label(seg, text="ステップ長 step (mm):").grid(row=0, column=0, sticky="e", padx=4, pady=4)
        self.var_step_mm = tk.StringVar(value=str(DEFAULT_STEP_MM))
        ttk.Entry(seg, textvariable=self.var_step_mm, width=8).grid(row=0, column=1, sticky="w")

        ttk.Label(seg, text="lookahead:").grid(row=0, column=2, sticky="e", padx=4)
        self.var_lookahead = tk.StringVar(value=str(DEFAULT_LOOKAHEAD))
        ttk.Entry(seg, textvariable=self.var_lookahead, width=8).grid(row=0, column=3, sticky="w")

        ttk.Label(seg, text="V_TCP (mm/s):").grid(row=1, column=0, sticky="e", padx=4)
        self.var_v_tcp = tk.StringVar(value=str(DEFAULT_V_TCP))
        ttk.Entry(seg, textvariable=self.var_v_tcp, width=8).grid(row=1, column=1, sticky="w")

        ttk.Label(seg, text="V_ORI (deg/s):").grid(row=1, column=2, sticky="e", padx=4)
        self.var_v_ori = tk.StringVar(value=str(DEFAULT_V_ORI))
        ttk.Entry(seg, textvariable=self.var_v_ori, width=8).grid(row=1, column=3, sticky="w")

        ttk.Label(seg, text="P_TCP (mm):").grid(row=2, column=0, sticky="e", padx=4, pady=4)
        self.var_p_tcp = tk.StringVar(value=str(DEFAULT_P_TCP))
        ttk.Entry(seg, textvariable=self.var_p_tcp, width=8).grid(row=2, column=1, sticky="w")

        ttk.Label(seg, text="P_ORI (deg):").grid(row=2, column=2, sticky="e", padx=4)
        self.var_p_ori = tk.StringVar(value=str(DEFAULT_P_ORI))
        ttk.Entry(seg, textvariable=self.var_p_ori, width=8).grid(row=2, column=3, sticky="w")

        ttk.Label(seg, text="Z_ORI (deg):").grid(row=2, column=4, sticky="e", padx=4)
        self.var_z_ori = tk.StringVar(value=str(DEFAULT_Z_ORI))
        ttk.Entry(seg, textvariable=self.var_z_ori, width=8).grid(row=2, column=5, sticky="w")

        # --- 終点 fine オプション ---
        fine = ttk.LabelFrame(self, text="終点のみFine収束（任意）"); fine.pack(padx=10, pady=6, fill="x")
        self.var_final_fine = tk.BooleanVar(value=DEFAULT_FINAL_FINE)
        ttk.Checkbutton(fine, text="終点だけ P_TCP を一時的に小さくする", variable=self.var_final_fine).grid(row=0, column=0, sticky="w", padx=6)
        ttk.Label(fine, text="終点 P_TCP (mm):").grid(row=0, column=1, sticky="e")
        self.var_final_p_tcp = tk.StringVar(value=str(DEFAULT_FINAL_P_TCP))
        ttk.Entry(fine, textvariable=self.var_final_p_tcp, width=8).grid(row=0, column=2, sticky="w", padx=4)

        # --- 下段：ボタン ---
        btns = ttk.Frame(self); btns.pack(padx=10, pady=10, fill="x")
        ttk.Button(btns, text="移動開始", command=self.on_start).pack(side="left", padx=6)
        ttk.Button(btns, text="停止", command=self.on_stop).pack(side="left", padx=6)

        # 自動接続と初期セット
        self.after(100, self._auto_connect_and_init)

    def _auto_connect_and_init(self):
        try:
            print("[INFO] Connecting...")
            self.client.connect(timeout=5.0)
            self.client.ping()
            # Tool / WObj
            self.client.set_wobj(identity=IDENTITY_WOBJ, txyz=WOBJ_TXYZ, rpy_deg=WOBJ_RPY)
            self.client.set_tool(TOOL_TXYZ, TOOL_RPY)
            # 初期スピード・ゾーン
            self._apply_speed_zone_from_gui()
            messagebox.showinfo("接続", "ロボットに接続しました。")
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

    def on_start(self):
        # 既存スレッドがあれば待つ
        if self.motion_thread and self.motion_thread.is_alive():
            messagebox.showwarning("実行中", "既に移動中です。停止してから再度実行してください。")
            return
        try:
            # 新規開始前に未読ACKを掃除（前回停止時の残り対策）
            self.client.flush_recv()

            x = float(self.vars["X(mm)"].get())
            y = float(self.vars["Y(mm)"].get())
            z = float(self.vars["Z(mm)"].get())
            r = float(self.vars["Roll(deg)"].get())
            p = float(self.vars["Pitch(deg)"].get())
            yaw = float(self.vars["Yaw(deg)"].get())

            step_mm = float(self.var_step_mm.get());   assert step_mm > 0
            lookahead = int(float(self.var_lookahead.get())); assert lookahead >= 1

            # 速度・ゾーンは開始時に反映（このときも未読ACKの影響を避けるため先にflush）
            self._apply_speed_zone_from_gui()

            final_fine = bool(self.var_final_fine.get())
            final_p_tcp = float(self.var_final_p_tcp.get())
            if final_fine and final_p_tcp <= 0:
                raise ValueError("終点P_TCPは正の値にしてください。")
        except Exception as e:
            messagebox.showerror("入力エラー", f"{e}")
            return

        self.stop_flag.clear()
        self.motion_thread = threading.Thread(
            target=self._run_motion,
            args=((x,y,z),(r,p,yaw), step_mm, lookahead, final_fine, final_p_tcp),
            daemon=True
        )
        self.motion_thread.start()

    def on_stop(self):
        self.stop_flag.set()

    def _run_motion(self, target_xyz, target_rpy, step_mm, lookahead, final_fine, final_p_tcp):
        try:
            # 現在TCP取得（この直前にも安全のため軽くflushしておく）
            self.client.flush_recv()
            cx, cy, cz, *_ = self.client.get_cart()

            # 経由点生成（姿勢は固定：target_rpy）
            via_xyz = make_via_points_by_step((cx,cy,cz), target_xyz, step_mm=step_mm)
            n = len(via_xyz)

            # 送信前にACKドレイン開始（中間点のACKを常時読み捨て）
            self.client.start_ack_drain()

            if n == 1:
                # ほぼ同一点 → ドレイン停止して終点ACK待ちのみ
                self.client.stop_ack_drain()
                self._move_final_with_optional_fine(via_xyz[-1], target_rpy, final_fine, final_p_tcp)
                print("[INFO] Motion sequence done.")
                return

            # 先行送信（中間点）：最初に lookahead 分（最後以外）を送る
            preload = min(lookahead, max(0, n-1))  # n-1 までが中間点
            idx_sent = -1
            for k in range(preload):
                if self.stop_flag.is_set():
                    self.client.stop_ack_drain()
                    self.client.flush_recv()
                    return
                vx, vy, vz = via_xyz[k]
                print(f"[PRELOAD] {k+1}/{n-1} -> ({vx:.2f},{vy:.2f},{vz:.2f}) send_only")
                self.client.moveL_send_only(vx, vy, vz, target_rpy)
                idx_sent = k

            # パイプライン: 中間点を送信し続ける
            while idx_sent < n-2:  # 最後の1点(n-1)はACK待ち用に残す
                if self.stop_flag.is_set():
                    print("[INFO] Stop requested. Halting further send.")
                    self.client.stop_ack_drain()
                    self.client.flush_recv()
                    return
                idx_sent += 1
                nxt = idx_sent
                if nxt < n-1:
                    vx, vy, vz = via_xyz[nxt]
                    print(f"[PIPE] {nxt+1}/{n-1} -> ({vx:.2f},{vy:.2f},{vz:.2f}) send_only")
                    self.client.moveL_send_only(vx, vy, vz, target_rpy)
                time.sleep(0.03)  # サーバが取りこぼさない程度の小休止

            if self.stop_flag.is_set():
                print("[INFO] Stopped before final segment.")
                self.client.stop_ack_drain()
                self.client.flush_recv()
                return

            # 最後の1点だけACK待ち（その直前でドレイン停止→ブロッキング受信へ復帰）
            self.client.stop_ack_drain()
            # 念のためここでも残りACKを掃除（ゾーン切替などの前にクリーンにする）
            self.client.flush_recv()
            self._move_final_with_optional_fine(via_xyz[-1], target_rpy, final_fine, final_p_tcp)
            print("[INFO] Motion sequence done.")

        except Exception as e:
            try:
                self.client.stop_ack_drain()
                self.client.flush_recv()
            except:
                pass
            messagebox.showerror("移動エラー", f"{e}")

    def _move_final_with_optional_fine(self, final_xyz, target_rpy, final_fine, final_p_tcp):
        if final_fine:
            # 現在のゾーン値を保存
            p_tcp = float(self.var_p_tcp.get())
            p_ori = float(self.var_p_ori.get())
            z_ori = float(self.var_z_ori.get())
            try:
                # 終点だけP_TCPを小さくしてピタ止め
                self.client.set_zone(final_p_tcp, p_ori, z_ori)
                self.client.moveL_ack(final_xyz[0], final_xyz[1], final_xyz[2], target_rpy,
                                      ack_timeout=MOVEL_ACK_TIMEOUT_SEC)
            finally:
                # 元に戻す
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
