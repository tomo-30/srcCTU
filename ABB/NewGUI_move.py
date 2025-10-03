# -*- coding: utf-8 -*-
# openabb_interruptible_gui.py
# GUIから open_abb(拡張 CASE 2/10/11) を操作:
#   - Send Target(10) : 非同期MoveLを投入（MOTIONタスクが実行）
#   - STOP(2)         : StopMoveを要求（即時停止）
#   - Get Pose(11)    : 移動中でも現在TCPを取得
#   - Auto Pose       : 300ms周期で現在TCPを自動更新
#
# 依存: 標準ライブラリ(tkinter, socket, math, threading, logging)

import socket, time, math, threading, logging, traceback
import tkinter as tk
from tkinter import ttk, messagebox

# ===== ログ設定 =====
logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
log = logging.getLogger("openabb_gui")

# ===== 接続先 =====
ROBOT_IP, ROBOT_PORT = "192.168.125.1", 5000

# ===== 初期ツール/ワークオブジェクト（必要に応じて変更）=====
TOOL_TX, TOOL_TY, TOOL_TZ = 0.0, 0.0, 0.0
TOOL_R, TOOL_P, TOOL_Y = 0.0, 0.0, 0.0  # deg ZYX
IDENTITY_WOBJ = True
WOBJ_WX, WOBJ_WY, WOBJ_WZ = 0.0, 0.0, 0.0
WOBJ_R, WOBJ_P, WOBJ_Y = 0.0, 0.0, 0.0  # deg ZYX

# ===== 速度/ゾーン =====
V_TCP, V_ORI = 50, 50
P_TCP, P_ORI, Z_ORI = 10, 10, 10

# ===== 目標初期値 =====
INIT_TARGET = [350.0, 0.0, 300.0, 0.0, 180.0, 0.0]  # x,y,z, r,p,y (mm/deg)

# ===== モニタ周期 =====
MONITOR_MS = 300


def fmt(n):
    return str(n) if isinstance(n, int) else f"{float(n):.6f}"

def rpy2quat(r, p, y):
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

def quat2rpy(qx, qy, qz, qw):
    R11 = 1 - 2*(qy*qy + qz*qz)
    R21 = 2*(qx*qy + qz*qw)
    R31 = 2*(qx*qz - qy*qw)
    R32 = 2*(qy*qz + qx*qw)
    R33 = 1 - 2*(qx*qx + qy*qy)
    yaw  = math.degrees(math.atan2(R21, R11))
    pitch = math.degrees(math.asin(-R31))
    roll = math.degrees(math.atan2(R32, R33))
    return roll, pitch, yaw


class OpenABB:
    def __init__(self, ip, port):
        self.ip, self.port = ip, port
        self.sock = None
        self.lock = threading.Lock()

    def connect(self, timeout=5.0):
        self.close()
        log.info(f"Connecting {self.ip}:{self.port}")
        s = socket.create_connection((self.ip, self.port), timeout=timeout)
        s.settimeout(3.0)
        self.sock = s
        log.info("Connected")

    def close(self):
        if self.sock:
            try: self.sock.close()
            except: pass
            self.sock = None

    def _send_cmd(self, parts, expect=None, pause=0.1):
        if not self.sock: raise RuntimeError("Not connected")
        msg = " ".join(fmt(p) for p in parts) + " #"
        with self.lock:
            log.info(f"SEND: {msg.strip()}")
            self.sock.sendall(msg.encode("ascii"))
            time.sleep(pause)
            data = self.sock.recv(2048).decode("ascii", errors="ignore").strip()
        log.info(f"RECV: {data}")
        toks = data.split()
        instr = int(toks[0]) if len(toks)>=1 and toks[0].lstrip("-").isdigit() else None
        ok    = int(toks[1]) if len(toks)>=2 and toks[1].lstrip("-").isdigit() else None
        if expect is not None and instr != expect:
            raise RuntimeError(f"ACK mismatch: expect {expect}, got {instr}, raw='{data}'")
        return instr, ok, toks

    # --- 既存互換 ---
    def ping(self):
        return self._send_cmd([0], expect=0)
    def set_tool(self, tx,ty,tz, r,p,y):
        qx,qy,qz,qw = rpy2quat(r,p,y)
        return self._send_cmd([6, tx,ty,tz, qx,qy,qz,qw], expect=6)
    def set_wobj_identity(self):
        # open_abbの単位姿勢は [1,0,0,0]
        return self._send_cmd([7, 0.0,0.0,0.0, 1.0,0.0,0.0,0.0], expect=7)
    def set_wobj(self, wx,wy,wz, r,p,y):
        qx,qy,qz,qw = rpy2quat(r,p,y)
        return self._send_cmd([7, wx,wy,wz, qx,qy,qz,qw], expect=7)
    def set_speed(self, v_tcp, v_ori):
        return self._send_cmd([8, v_tcp, v_ori], expect=8)
    def set_zone_simple(self, p_tcp, p_ori, z_ori):
        return self._send_cmd([9, 0, p_tcp, p_ori, z_ori], expect=9)

    # --- 新規プロトコル ---
    def stop(self):
        return self._send_cmd([2], expect=2)
    def goto_xyzrpy_async(self, x,y,z, r,p,yw):
        qx,qy,qz,qw = rpy2quat(r,p,yw)
        return self._send_cmd([10, x,y,z, qx,qy,qz,qw], expect=10)
    def get_pose(self):
        # 拡張(11) を優先して使う
        instr, ok, toks = self._send_cmd([11], expect=11)
        if ok != 1 or len(toks) < 9:
            raise RuntimeError("GETPOSE failed")
        x = float(toks[2]); y = float(toks[3]); z = float(toks[4])
        qx = float(toks[5]); qy = float(toks[6]); qz = float(toks[7]); qw = float(toks[8])
        r,p,yw = quat2rpy(qx,qy,qz,qw)
        return x,y,z,r,p,yw


class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("ABB IRB1200 Interruptible GUI (open_abb ext)")
        self.geometry("620x320")
        self.resizable(False, False)

        self.client = OpenABB(ROBOT_IP, ROBOT_PORT)
        self.connected = False
        self.auto_pose = tk.BooleanVar(value=True)

        self._build_ui()
        threading.Thread(target=self._init_connect, daemon=True).start()
        self.after(MONITOR_MS, self._auto_poll)

    def _build_ui(self):
        pad = {'padx': 8, 'pady': 5}
        self.lbl_status = ttk.Label(self, text="Status: connecting...", foreground="blue")
        self.lbl_status.grid(row=0, column=0, columnspan=6, sticky="w", **pad)

        # 現在表示
        ttk.Label(self, text="Current TCP [mm/deg]").grid(row=1, column=0, columnspan=6, sticky="w", **pad)
        self.cur_vars = [tk.StringVar(value="---") for _ in range(6)]
        labels = ["x","y","z","roll","pitch","yaw"]
        for i, name in enumerate(labels):
            ttk.Label(self, text=f"{name}:").grid(row=2 + i//3, column=(i%3)*2+0, sticky="e", **pad)
            ttk.Label(self, textvariable=self.cur_vars[i], width=12).grid(row=2 + i//3, column=(i%3)*2+1, sticky="w", **pad)

        # 目標入力
        ttk.Label(self, text="Target XYZRPY [mm/deg]").grid(row=4, column=0, columnspan=6, sticky="w", **pad)
        self.ent_vars = []
        defaults = INIT_TARGET
        for i, name in enumerate(labels):
            ttk.Label(self, text=f"{name}:").grid(row=5 + i//3, column=(i%3)*2+0, sticky="e", **pad)
            var = tk.StringVar(value=str(defaults[i]))
            ent = ttk.Entry(self, textvariable=var, width=12)
            ent.grid(row=5 + i//3, column=(i%3)*2+1, sticky="w", **pad)
            self.ent_vars.append(var)

        # ボタン群
        btn_frame = ttk.Frame(self)
        btn_frame.grid(row=7, column=0, columnspan=6, sticky="w", **pad)

        ttk.Button(btn_frame, text="Send Target (GOTO)", command=self.on_send_target).grid(row=0, column=0, **pad)
        ttk.Button(btn_frame, text="STOP (StopMove)", command=self.on_stop).grid(row=0, column=1, **pad)
        ttk.Button(btn_frame, text="Get Pose (now)", command=self.on_get_pose).grid(row=0, column=2, **pad)
        ttk.Checkbutton(btn_frame, text="Auto Pose", variable=self.auto_pose).grid(row=0, column=3, **pad)

    def _set_status(self, text, color="black"):
        def _upd(): self.lbl_status.config(text=f"Status: {text}", foreground=color)
        self.after(0, _upd)

    def _init_connect(self):
        try:
            self._set_status("connecting...", "blue")
            self.client.connect(timeout=5.0)
            self.client.ping()
            # Tool/WObj/Speed/Zone 初期化
            self.client.set_tool(TOOL_TX,TOOL_TY,TOOL_TZ, TOOL_R,TOOL_P,TOOL_Y)
            if IDENTITY_WOBJ: self.client.set_wobj_identity()
            else: self.client.set_wobj(WOBJ_WX,WOBJ_WY,WOBJ_WZ, WOBJ_R,WOBJ_P,WOBJ_Y)
            self.client.set_speed(V_TCP, V_ORI)
            self.client.set_zone_simple(P_TCP, P_ORI, Z_ORI)
            self.connected = True
            self._set_status("connected", "green")
            log.info("Init done")
        except Exception as e:
            self.connected = False
            self._set_status(f"connect/init failed: {e}", "red")
            log.error("init failed:\n" + traceback.format_exc())

    def on_send_target(self):
        if not self.connected:
            messagebox.showerror("Error", "Not connected")
            return
        try:
            vals = [float(v.get()) for v in self.ent_vars]  # x y z r p y
            x,y,z,r,p,yw = vals
        except ValueError:
            messagebox.showerror("Error", "Invalid numeric input")
            return

        def _worker():
            try:
                instr, ok, _ = self.client.goto_xyzrpy_async(x,y,z,r,p,yw)
                if ok == 1: self._set_status("GOTO accepted", "green")
                else: self._set_status("GOTO rejected", "orange")
            except Exception as e:
                self._set_status(f"GOTO error: {e}", "red")
                log.error("GOTO error:\n" + traceback.format_exc())
        threading.Thread(target=_worker, daemon=True).start()

    def on_stop(self):
        if not self.connected: return
        def _worker():
            try:
                instr, ok, _ = self.client.stop()
                if ok == 1: self._set_status("STOP acknowledged", "orange")
                else: self._set_status("STOP rejected", "red")
            except Exception as e:
                self._set_status(f"STOP error: {e}", "red")
                log.error("STOP error:\n" + traceback.format_exc())
        threading.Thread(target=_worker, daemon=True).start()

    def on_get_pose(self):
        if not self.connected: return
        def _worker():
            try:
                x,y,z,r,p,yw = self.client.get_pose()
                for var,val in zip(self.cur_vars, [x,y,z,r,p,yw]):
                    var.set(f"{val:.3f}")
                self._set_status("pose updated", "black")
            except Exception as e:
                self._set_status(f"GETPOSE error: {e}", "red")
                log.error("GETPOSE error:\n" + traceback.format_exc())
        threading.Thread(target=_worker, daemon=True).start()

    def _auto_poll(self):
        if self.connected and self.auto_pose.get():
            try:
                x,y,z,r,p,yw = self.client.get_pose()
                for var,val in zip(self.cur_vars, [x,y,z,r,p,yw]):
                    var.set(f"{val:.3f}")
            except Exception as e:
                self._set_status(f"auto GETPOSE err: {e}", "red")
                log.error("auto GETPOSE error:\n" + traceback.format_exc())
        self.after(MONITOR_MS, self._auto_poll)


if __name__ == "__main__":
    app = App()
    app.mainloop()
