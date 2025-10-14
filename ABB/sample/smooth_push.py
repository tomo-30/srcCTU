# -*- coding: utf-8 -*-
# gui_move_tcp_openabb_fixed_pose_eps.py
# 目的:
#   open_abb (CASE 0/1/3/6/7/8/9) を用い，
#   ・現在位置→目標XYZへ MoveL（分割）で移動
#   ・移動“中”は手先姿勢を r,p,y = (ROLL_BIAS, 180 - EPS, 0) に完全固定
#   ・到達後に必要なら p=180° へその場スナップ（静止中）
#   ・qと−qの符号反転（等価姿勢）を連続化して抑止
#   ・移動中のTCPポーリングは間引きで通信競合を低減
#
# 実行:
#   python gui_move_tcp_openabb_fixed_pose_eps.py

import socket
import time
import math
import threading
import tkinter as tk
from tkinter import ttk, messagebox
import logging, traceback

# ===== ロギング =====
logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
log = logging.getLogger("openabb_fixed_pose_eps")

# ===== 接続先 =====
ROBOT_IP, ROBOT_PORT = "192.168.125.1", 5000  # 環境に合わせて変更

# ===== Tool / WObj =====
TOOL_TX, TOOL_TY, TOOL_TZ = 0.0, 0.0, 0.0
TOOL_R_DEG, TOOL_P_DEG, TOOL_Y_DEG = 0.0, 0.0, 0.0  # ZYX
IDENTITY_WOBJ = True
WOBJ_WX, WOBJ_WY, WOBJ_WZ = 0.0, 0.0, 0.0
WOBJ_R_DEG, WOBJ_P_DEG, WOBJ_Y_DEG = 0.0, 0.0, 0.0  # ZYX

# ===== 速度・ゾーン =====
V_TCP, V_ORI = 40, 40
P_TCP, P_ORI, Z_ORI = 10, 10, 10

# ===== モニタ周期 =====
MONITOR_MS_IDLE   = 300
MONITOR_MS_MOVING = 700  # 送受信競合を避けるため移動中は間引き

# ===== 分割MoveL =====
STEP_MM = 5.0                 # 1ステップ距離(mm)。安定化には 3〜4 も有効
SLEEP_BETWEEN_STEPS = 0.0     # ステップ間ウェイト(s)

# ===== 特異点回避のための固定姿勢（移動中）=====
#   p=180°は手首特異点なので，移動中は p = 180 - EPS で固定
#   roll に少しバイアスを与えると（J4/J6直線特異から離せる），より安定
EPS_DEG = 1.0                 # 推奨 0.5〜2.0
ROLL_BIAS_DEG = 30.0          # 推奨 0,15,30,45 のいずれかで試す
MOVE_R_DEG, MOVE_P_DEG, MOVE_Y_DEG = ROLL_BIAS_DEG, 180.0 - EPS_DEG, 0.0

# 到達後に「厳密180°」へスナップするか（静止中なので安全）
FINAL_SNAP_TO_EXACT_180 = True

# ===== 初期XYZ =====
INIT_TARGET_XYZ = [350.0, 0.0, 300.0]

# ---------- 数学ユーティリティ ----------
def fmt(n): return str(n) if isinstance(n,int) else f"{float(n):.6f}"
def _deg2rad(d): return d*math.pi/180.0
def _rad2deg(r): return r*180.0/math.pi

def rpy_deg_to_quat(r_deg, p_deg, y_deg):
    """ZYX RPY(deg) -> [qx,qy,qz,qw]（ABB順）"""
    rx, ry, rz = map(_deg2rad, (r_deg, p_deg, y_deg))
    cx, sx = math.cos(rx/2), math.sin(rx/2)
    cy, sy = math.cos(ry/2), math.sin(ry/2)
    cz, sz = math.cos(rz/2), math.sin(rz/2)
    qw = cz*cy*cx + sz*sy*sx
    qx = cz*cy*sx - sz*sy*cx
    qy = cz*sy*cx + sz*cy*sx
    qz = sz*cy*cx - cz*sy*sx
    n = math.sqrt(qw*qw + qx*qx + qy*qy + qz*qz) or 1.0
    return [qx/n, qy/n, qz/n, qw/n]

def quat_to_rpy_deg(qx,qy,qz,qw):
    """[qx,qy,qz,qw] -> ZYX RPY(deg)"""
    R11 = 1 - 2*(qy*qy + qz*qz)
    R21 = 2*(qx*qy + qz*qw)
    R31 = 2*(qx*qz - qy*qw)
    R32 = 2*(qy*qz + qx*qw)
    R33 = 1 - 2*(qx*qx + qy*qy)
    yaw   = _rad2deg(math.atan2(R21, R11))
    pitch = _rad2deg(math.asin(-R31))
    roll  = _rad2deg(math.atan2(R32, R33))
    return roll, pitch, yaw

def quat_dot(a,b): return a[0]*b[0]+a[1]*b[1]+a[2]*b[2]+a[3]*b[3]
def quat_neg(q):   return [-q[0],-q[1],-q[2],-q[3]]

# ---------- 姿勢連続化（符号反転の抑止） ----------
class OrientationKeeper:
    def __init__(self):
        self.lock = threading.Lock()
        self.last_q = None  # [qx,qy,qz,qw]

    def init_from_tcp(self, qx,qy,qz,qw):
        with self.lock:
            self.last_q = [qx,qy,qz,qw]

    def stabilize(self, q):
        """lastと内積が負なら q→-q（姿勢自体は同一）"""
        with self.lock:
            if self.last_q is None:
                self.last_q = q[:]
                return q
            if quat_dot(self.last_q, q) < 0.0:
                q = quat_neg(q)
            self.last_q = q[:]
            return q

# ---------- open_abb クライアント ----------
class OpenABBRobotClient:
    def __init__(self, ip, port):
        self.ip = ip; self.port = port
        self.sock = None
        self.lock = threading.Lock()
        self.recv_timeout = 4.0
        self.send_pause   = 0.12   # 送信後に少し待って受信

    def connect(self, timeout=5.0):
        self.close()
        log.info(f"Connecting {self.ip}:{self.port}")
        s = socket.create_connection((self.ip, self.port), timeout=timeout)
        s.settimeout(self.recv_timeout)
        self.sock = s
        log.info("Connected")

    def close(self):
        if self.sock:
            try: self.sock.close()
            except: pass
            self.sock = None

    def _send_cmd(self, parts, expect=None, retries=1):
        if not self.sock:
            raise RuntimeError("Not connected")
        msg = " ".join(fmt(p) for p in parts) + " #"
        for attempt in range(retries+1):
            try:
                with self.lock:
                    self.sock.sendall(msg.encode("ascii"))
                    time.sleep(self.send_pause)
                    data = self.sock.recv(2048).decode("ascii", errors="ignore").strip()
                log.info(f"SEND: {msg.strip()}   RECV: {data}")
                toks = data.split()
                instr = int(toks[0]) if len(toks)>=1 and toks[0].lstrip("-").isdigit() else None
                ok    = int(toks[1]) if len(toks)>=2 and toks[1].lstrip("-").isdigit() else None
                if expect is not None and instr != expect:
                    raise RuntimeError(f"ACK mismatch: expect {expect}, got {instr}, raw='{data}'")
                return instr, ok, toks
            except (socket.timeout, TimeoutError):
                log.warning(f"recv timeout on '{msg.strip()}', attempt {attempt+1}")
                if attempt >= retries:
                    raise
                time.sleep(0.2)
        raise RuntimeError("unreachable")

    @staticmethod
    def _parse_case3_tokens(toks):
        if len(toks) >= 9:
            return (float(toks[2]), float(toks[3]), float(toks[4]),
                    float(toks[5]), float(toks[6]), float(toks[7]), float(toks[8]))
        raise ValueError("CASE3 reply too short")

    # ---- open_abb CASE wrappers ----
    def ping(self): return self._send_cmd([0], expect=0)

    def set_wobj(self, identity=True, wx=0, wy=0, wz=0, r=0, p=0, y=0):
        if identity:
            # open_abbの単位WObj（クォータニオン = [1,0,0,0]）
            return self._send_cmd([7, 0.0,0.0,0.0, 1.0,0.0,0.0,0.0], expect=7)
        else:
            wqx,wqy,wqz,wqw = rpy_deg_to_quat(r,p,y)
            return self._send_cmd([7, wx,wy,wz, wqx,wqy,wqz,wqw], expect=7)

    def set_tool(self, tx,ty,tz, r,p,y):
        tqx,tqy,tqz,tqw = rpy_deg_to_quat(r,p,y)
        return self._send_cmd([6, tx,ty,tz, tqx,tqy,tqz,tqw], expect=6)

    def set_speed_zone(self, v_tcp, v_ori, p_tcp, p_ori, z_ori):
        self._send_cmd([8, v_tcp, v_ori], expect=8)
        return self._send_cmd([9, 0, p_tcp, p_ori, z_ori], expect=9)

    def get_tcp(self):
        instr, ok, toks = self._send_cmd([3], expect=3, retries=1)
        if ok == 1:
            x,y,z,qx,qy,qz,qw = self._parse_case3_tokens(toks)
            return True, (x,y,z,qx,qy,qz,qw)
        return False, None

    def movel_quat(self, x,y,z, q):
        qx,qy,qz,qw = q
        instr, ok, _ = self._send_cmd([1, x,y,z, qx,qy,qz,qw], expect=1, retries=1)
        return ok == 1

# ---------- GUI本体（XYZ入力のみ。姿勢は移動中固定） ----------
class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("ABB IRB1200 - MoveL with fixed pose (avoid 180° during motion)")
        self.geometry("560x320")
        self.resizable(False, False)

        self.client = OpenABBRobotClient(ROBOT_IP, ROBOT_PORT)
        self.okeeper = OrientationKeeper()

        self.connected = False
        self.in_motion = False
        self.pending_target = None
        self.state_lock = threading.Lock()

        # 移動中固定姿勢 & 停止後スナップ先（必要なら）
        self.q_move  = rpy_deg_to_quat(MOVE_R_DEG, MOVE_P_DEG, MOVE_Y_DEG)
        self.q_exact = rpy_deg_to_quat(ROLL_BIAS_DEG, 180.0,            0.0)

        self._build_widgets()
        threading.Thread(target=self._init_robot, daemon=True).start()
        self.after(MONITOR_MS_IDLE, self._poll_tcp)

    def _build_widgets(self):
        pad = {'padx': 8, 'pady': 5}

        self.lbl_status = ttk.Label(self, text="Status: connecting...", foreground="blue")
        self.lbl_status.grid(row=0, column=0, columnspan=6, sticky="w", **pad)

        # 現在TCP表示
        ttk.Label(self, text="Current TCP [mm/deg]").grid(row=1, column=0, columnspan=6, sticky="w", **pad)
        self.cur_vars = [tk.StringVar(value="---") for _ in range(6)]
        for i, name in enumerate(["x","y","z","roll","pitch","yaw"]):
            ttk.Label(self, text=f"{name}:").grid(row=2 + i//3, column=(i%3)*2+0, sticky="e", **pad)
            ttk.Label(self, textvariable=self.cur_vars[i], width=12).grid(row=2 + i//3, column=(i%3)*2+1, sticky="w", **pad)

        # 目標XYZ入力（RPYは固定で入力なし）
        ttk.Label(self, text=f"Target XYZ [mm]  (Moving pose LOCKED at r,p,y=({ROLL_BIAS_DEG:.1f},{180.0-EPS_DEG:.1f},0.0))").grid(row=4, column=0, columnspan=6, sticky="w", **pad)
        self.ent_x = tk.StringVar(value=str(INIT_TARGET_XYZ[0]))
        self.ent_y = tk.StringVar(value=str(INIT_TARGET_XYZ[1]))
        self.ent_z = tk.StringVar(value=str(INIT_TARGET_XYZ[2]))

        ttk.Label(self, text="x:").grid(row=5, column=0, sticky="e", **pad)
        ttk.Entry(self, textvariable=self.ent_x, width=12).grid(row=5, column=1, sticky="w", **pad)
        ttk.Label(self, text="y:").grid(row=5, column=2, sticky="e", **pad)
        ttk.Entry(self, textvariable=self.ent_y, width=12).grid(row=5, column=3, sticky="w", **pad)
        ttk.Label(self, text="z:").grid(row=5, column=4, sticky="e", **pad)
        ttk.Entry(self, textvariable=self.ent_z, width=12).grid(row=5, column=5, sticky="w", **pad)

        self.btn_send = ttk.Button(self, text="Send MoveL (fixed pose, avoid 180°)", command=self._on_send)
        self.btn_send.grid(row=7, column=0, columnspan=6, **pad)

    def _set_status(self, text, color):
        def _upd(): self.lbl_status.config(text=f"Status: {text}", foreground=color)
        self.after(0, _upd)

    # ---- 初期化 ----
    def _init_robot(self):
        try:
            self._set_status("connecting...", "blue")
            self.client.connect(timeout=5.0)
            self.client.ping()
            self.client.set_wobj(IDENTITY_WOBJ, WOBJ_WX,WOBJ_WY,WOBJ_WZ, WOBJ_R_DEG,WOBJ_P_DEG,WOBJ_Y_DEG)
            self.client.set_tool(TOOL_TX,TOOL_TY,TOOL_TZ, TOOL_R_DEG,TOOL_P_DEG,TOOL_Y_DEG)
            self.client.set_speed_zone(V_TCP, V_ORI, P_TCP, P_ORI, Z_ORI)

            ok, tcp = self.client.get_tcp()
            if ok:
                _,_,_, qx,qy,qz,qw = tcp
                # 直近姿勢で連続化の基準を作り，固定姿勢qも連続化しておく
                self.okeeper.init_from_tcp(qx,qy,qz,qw)
                self.q_move  = self.okeeper.stabilize(self.q_move)
                self.q_exact = self.okeeper.stabilize(self.q_exact)
            else:
                # 取得に失敗しても q_move / q_exact はそのまま使用
                pass

            self.connected = True
            self._set_status("connected", "green")
            log.info("Robot init done")
        except Exception as e:
            self.connected = False
            self._set_status(f"connect/init failed: {e}", "red")
            log.error("Init failed:\n" + traceback.format_exc())

    # ---- ポーリング ----
    def _poll_tcp(self):
        period = MONITOR_MS_MOVING if self.in_motion else MONITOR_MS_IDLE
        if self.connected:
            if self.in_motion:
                # 移動中は間引き（コントローラとの受信競合を避ける）
                self.after(period, self._poll_tcp)
                return
            try:
                ok, tcp = self.client.get_tcp()
                if ok:
                    x,y,z,qx,qy,qz,qw = tcp
                    r,p,yw = quat_to_rpy_deg(qx,qy,qz,qw)
                    for i,v in enumerate([x,y,z,r,p,yw]):
                        self.cur_vars[i].set(f"{v:.3f}")
                else:
                    self._set_status("get_tcp NG", "orange")
            except Exception as e:
                self._set_status(f"poll error: {e}", "red")
                log.error("Poll error:\n" + traceback.format_exc())
        self.after(period, self._poll_tcp)

    # ---- 送信 ----
    def _on_send(self):
        if not self.connected:
            messagebox.showerror("Error", "Not connected to robot.")
            return
        try:
            x = float(self.ent_x.get())
            y = float(self.ent_y.get())
            z = float(self.ent_z.get())
        except ValueError:
            messagebox.showerror("Error", "Invalid numeric input.")
            return

        with self.state_lock:
            if self.in_motion:
                self.pending_target = (x,y,z)
                self._set_status("queued latest target", "orange")
                return
            self.in_motion = True

        threading.Thread(target=self._do_movel_fixed_pose, args=((x,y,z),), daemon=True).start()

    # ---- 固定姿勢での分割MoveL本体 ----
    def _do_movel_fixed_pose(self, target_xyz):
        try:
            ok, tcp = self.client.get_tcp()
            if not ok:
                raise RuntimeError("get_tcp failed at start")
            sx,sy,sz, _,_,_,_ = tcp

            tx,ty,tz = target_xyz
            dx,dy,dz = tx-sx, ty-sy, tz-sz
            dist = math.sqrt(dx*dx + dy*dy + dz*dz)
            steps = max(1, int(math.ceil(dist / max(1e-6, STEP_MM))))

            self._set_status(f"MoveL(seg) with fixed rpy=({ROLL_BIAS_DEG:.1f},{180.0-EPS_DEG:.1f},0.0)...", "blue")

            # 以降，すべてのステップで同じクォータニオンを送る（姿勢固定）
            q_use = self.q_move[:]  # すでにstabilize済み

            for i in range(1, steps+1):
                ax = sx + dx*i/steps
                ay = sy + dy*i/steps
                az = sz + dz*i/steps
                ok_m = self.client.movel_quat(ax, ay, az, q_use)
                if not ok_m:
                    log.warning("MoveL(seg) ack != 1")
                if SLEEP_BETWEEN_STEPS > 0.0:
                    time.sleep(SLEEP_BETWEEN_STEPS)

            # 到達後のみ，厳密180°へスナップ（任意）
            if FINAL_SNAP_TO_EXACT_180:
                self._set_status("Snap to exact p=180° at rest...", "blue")
                self.client.movel_quat(tx, ty, tz, self.q_exact)

            # 簡易到達確認
            time.sleep(0.3)
            ok2, tcp2 = self.client.get_tcp()
            if ok2:
                mx,my,mz, qx,qy,qz,qw = tcp2
                pos_err = math.sqrt((mx-tx)**2+(my-ty)**2+(mz-tz)**2)
                r,p,yw = quat_to_rpy_deg(qx,qy,qz,qw)
                self._set_status(f"pos_err={pos_err:.2f} mm, rpy=({r:.2f},{p:.2f},{yw:.2f})", "green")

        except Exception as e:
            self._set_status(f"MoveL error: {e}", "red")
            log.error("MoveL error:\n" + traceback.format_exc())
        finally:
            next_target = None
            with self.state_lock:
                if self.pending_target is not None:
                    next_target = self.pending_target
                    self.pending_target = None
                else:
                    self.in_motion = False
            if next_target is not None:
                self._do_movel_fixed_pose(next_target)
            else:
                with self.state_lock:
                    self.in_motion = False

if __name__ == "__main__":
    app = App()
    app.mainloop()
