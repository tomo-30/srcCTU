# -*- coding: utf-8 -*-
# move_gui_segmented.py
# 目的:
#   ・GUIでXYZRPY(mm/deg)の目標姿勢を入力し，MoveLで到達
#   ・現在位置→目標位置を「分割ステップ長(mm)」または「分割数(N)」で経由点生成し逐次MoveL
#   ・各MoveLのACK待ち(受信)だけタイムアウト60秒（A案）
#   ・停止ボタンで分割間に安全に停止（経由点で介入）
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

# ====== 既定パラメータ（必要最小限） ======
# Tool / WObj は単位姿勢（ツール先＝フランジ，WObj＝ベース）
TOOL_TXYZ = (0.0, 0.0, 0.0)
TOOL_RPY  = (0.0, 0.0, 0.0)
IDENTITY_WOBJ = True
WOBJ_TXYZ = (0.0, 0.0, 0.0)
WOBJ_RPY  = (0.0, 0.0, 0.0)

# 速度・ゾーン（必要最低限）
V_TCP, V_ORI = 50, 50
P_TCP, P_ORI, Z_ORI = 10, 10, 10

# 既定の分割パラメータ
DEFAULT_STEP_MM = 60.0
DEFAULT_SPLIT_N = 8

# MoveL のみACK待ちを長めに（A案）
MOVEL_ACK_TIMEOUT_SEC = 60.0

# ソケット共通の短いタイムアウト（PingやGet、Set系）
DEFAULT_RECV_TIMEOUT = 3.0

# ------------------------------------------------------------

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
        self.sock = None

    def connect(self, timeout=5.0):
        self.sock = socket.create_connection((self.host, self.port), timeout=timeout)
        self.sock.settimeout(DEFAULT_RECV_TIMEOUT)
        print(f"[INFO] Connected {self.host}:{self.port}")

    def close(self):
        try:
            if self.sock:
                self.sock.close()
        finally:
            self.sock = None

    def _send_and_recv(self, parts, recv_timeout=None, expect=None, pause=0.15):
        if not self.sock:
            raise RuntimeError("Socket not connected")

        msg = " ".join(fmt(p) for p in parts) + " #"
        self.sock.sendall(msg.encode("ascii"))
        time.sleep(pause)

        old_to = self.sock.gettimeout()
        if recv_timeout is not None:
            self.sock.settimeout(recv_timeout)
        try:
            data = self.sock.recv(4096).decode("ascii", errors="ignore").strip()
        finally:
            if recv_timeout is not None:
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

    def moveL(self, x, y, z, rpy_deg, ack_timeout=MOVEL_ACK_TIMEOUT_SEC):
        qx, qy, qz, qw = rpy_deg_to_quat(*rpy_deg)
        return self._send_and_recv([1, x, y, z, qx, qy, qz, qw],
                                   recv_timeout=ack_timeout, expect=1)

# ---- 経由点生成 ----
def make_via_points_by_step(curr_xyz, target_xyz, step_mm):
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

def make_via_points_by_count(curr_xyz, target_xyz, n_split):
    # n_split 分割 → 経由点は n_split 個（最後は目標）
    cx, cy, cz = curr_xyz
    tx, ty, tz = target_xyz
    dx, dy, dz = tx - cx, ty - cy, tz - cz
    if n_split < 1:
        return [target_xyz]
    via = []
    for i in range(1, n_split + 1):
        s = i / n_split
        via.append((cx + dx*s, cy + dy*s, cz + dz*s))
    via[-1] = (tx, ty, tz)
    return via

# ============================================================
# GUI
# ============================================================

class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("open_abb MoveL (segmented)")
        self.geometry("520x320")

        self.client = RobotClient(ROBOT_IP, ROBOT_PORT)
        self.stop_flag = threading.Event()
        self.motion_thread = None

        # --- 上段：目標入力 ---
        frm = ttk.Frame(self); frm.pack(padx=10, pady=10, fill="x")

        self.vars = {}
        labels = ["X(mm)","Y(mm)","Z(mm)","Roll(deg)","Pitch(deg)","Yaw(deg)"]
        defaults = [400.0, 0.0, 100.0, 0.0, 180.0, 0.0]
        for i,(lab,defv) in enumerate(zip(labels, defaults)):
            ttk.Label(frm, text=lab, width=10).grid(row=i//3, column=(i%3)*2, sticky="e", padx=4, pady=4)
            v = tk.StringVar(value=str(defv)); self.vars[lab]=v
            ttk.Entry(frm, textvariable=v, width=10).grid(row=i//3, column=(i%3)*2+1, sticky="w")

        # --- 中段：分割パラメータ ---
        seg = ttk.LabelFrame(self, text="分割パラメータ"); seg.pack(padx=10, pady=6, fill="x")

        self.split_mode = tk.StringVar(value="step")  # "step" or "count"
        rb1 = ttk.Radiobutton(seg, text="距離ステップ指定（mm）", value="step", variable=self.split_mode)
        rb2 = ttk.Radiobutton(seg, text="分割数指定（N）", value="count", variable=self.split_mode)
        rb1.grid(row=0, column=0, sticky="w", padx=6, pady=4)
        rb2.grid(row=1, column=0, sticky="w", padx=6, pady=4)

        ttk.Label(seg, text="ステップ長 mm:").grid(row=0, column=1, sticky="e")
        self.var_step_mm = tk.StringVar(value=str(DEFAULT_STEP_MM))
        ttk.Entry(seg, textvariable=self.var_step_mm, width=8).grid(row=0, column=2, sticky="w", padx=4)

        ttk.Label(seg, text="分割数 N:").grid(row=1, column=1, sticky="e")
        self.var_split_n = tk.StringVar(value=str(DEFAULT_SPLIT_N))
        ttk.Entry(seg, textvariable=self.var_split_n, width=8).grid(row=1, column=2, sticky="w", padx=4)

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
            # Tool / WObj / Speed / Zone
            self.client.set_wobj(identity=IDENTITY_WOBJ, txyz=WOBJ_TXYZ, rpy_deg=WOBJ_RPY)
            self.client.set_tool(TOOL_TXYZ, TOOL_RPY)
            self.client.set_speed(V_TCP, V_ORI)
            self.client.set_zone(P_TCP, P_ORI, Z_ORI)
            messagebox.showinfo("接続", "ロボットに接続しました。")
        except Exception as e:
            messagebox.showerror("接続エラー", f"{e}")

    def on_start(self):
        if self.motion_thread and self.motion_thread.is_alive():
            messagebox.showwarning("実行中", "既に移動中です。停止してから再度実行してください。")
            return
        try:
            x = float(self.vars["X(mm)"].get())
            y = float(self.vars["Y(mm)"].get())
            z = float(self.vars["Z(mm)"].get())
            r = float(self.vars["Roll(deg)"].get())
            p = float(self.vars["Pitch(deg)"].get())
            yaw = float(self.vars["Yaw(deg)"].get())
        except ValueError:
            messagebox.showerror("入力エラー", "XYZRPYは数値を入力してください。")
            return

        # 分割パラメータ取得
        mode = self.split_mode.get()
        if mode == "step":
            try:
                step_mm = float(self.var_step_mm.get())
                if step_mm <= 0:
                    raise ValueError
            except ValueError:
                messagebox.showerror("入力エラー", "ステップ長(mm)は正の数を入力してください。")
                return
            split_cfg = ("step", step_mm)
        else:
            try:
                split_n = int(float(self.var_split_n.get()))
                if split_n < 1:
                    raise ValueError
            except ValueError:
                messagebox.showerror("入力エラー", "分割数Nは1以上の整数を入力してください。")
                return
            split_cfg = ("count", split_n)

        self.stop_flag.clear()
        self.motion_thread = threading.Thread(
            target=self._run_motion, args=((x,y,z),(r,p,yaw), split_cfg), daemon=True
        )
        self.motion_thread.start()

    def on_stop(self):
        self.stop_flag.set()

    def _run_motion(self, target_xyz, target_rpy, split_cfg):
        try:
            # 現在TCP取得
            cx, cy, cz, cqx, cqy, cqz, cqw = self.client.get_cart()
            print(f"[CURR] ({cx:.3f},{cy:.3f},{cz:.3f})")

            # 経由点列を作成（姿勢は固定：target_rpy）
            mode, val = split_cfg
            if mode == "step":
                via_xyz = make_via_points_by_step((cx,cy,cz), target_xyz, step_mm=val)
            else:
                via_xyz = make_via_points_by_count((cx,cy,cz), target_xyz, n_split=val)

            # 経由点ごとに MoveL（ACKは最大60秒待ち：A案）
            for i,(vx,vy,vz) in enumerate(via_xyz, start=1):
                if self.stop_flag.is_set():
                    print("[INFO] Stopped before sending next segment.")
                    break
                print(f"[INFO] Segment {i}/{len(via_xyz)} -> ({vx:.2f},{vy:.2f},{vz:.2f})")
                self.client.moveL(vx, vy, vz, target_rpy, ack_timeout=MOVEL_ACK_TIMEOUT_SEC)
                if self.stop_flag.is_set():
                    print("[INFO] Stopped after segment reach.")
                    break

            print("[INFO] Motion sequence done.")
        except Exception as e:
            messagebox.showerror("移動エラー", f"{e}")

    def destroy(self):
        try:
            self.client.close()
        except:
            pass
        super().destroy()

if __name__ == "__main__":
    app = App()
    app.mainloop()
