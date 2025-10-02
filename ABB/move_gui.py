# -*- coding: utf-8 -*-
# gui_move_tcp_openabb_queue_latest.py
# 目的:
#   open_abb (CASE 0/1/3/6/7/8/9) を用い，
#   GUIからTCP目標(x,y,z,roll,pitch,yaw)[mm/deg]を指定→MoveL．
#   現在TCPは移動中も300ms周期で取得表示．
#   送信中に新たな目標が来たら「最新1件だけをキュー」して前動作終了後に即MoveL．
#
# 変更点:
#   ・logging/tracebackでターミナルへ詳細ログ出力（INFO/ERROR）
#   ・最新目標キュー(pending_target)機構を追加
#   ・in_motionフラグで送信の直列化，ステータス可視化
#
# 実行: python gui_move_tcp_openabb_queue_latest.py

import socket
import time
import math
import threading
import tkinter as tk
from tkinter import ttk, messagebox
import logging, traceback

# ===== ロギング設定 =====
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s"
)
log = logging.getLogger("openabb_gui")

# ====== 通信先設定 ======
ROBOT_IP, ROBOT_PORT = "192.168.125.1", 5000  # 必要に応じて変更

# ====== Tool設定（フランジ→TCP）: mm/deg ======
TOOL_TX, TOOL_TY, TOOL_TZ = 0.0, 0.0, 0.0
TOOL_R_DEG, TOOL_P_DEG, TOOL_Y_DEG = 0.0, 0.0, 0.0  # ZYX順

# ====== WObj設定（ベース→WObj）: mm/deg ======
IDENTITY_WOBJ = True
WOBJ_WX,  WOBJ_WY,  WOBJ_WZ  = 0.0, 0.0, 0.0
WOBJ_R_DEG, WOBJ_P_DEG, WOBJ_Y_DEG = 0.0, 0.0, 0.0  # ZYX順

# ====== 速度・ゾーン ======
V_TCP, V_ORI = 50, 50
P_TCP, P_ORI, Z_ORI = 10, 10, 10

# ====== モニタ更新周期(ms) ======
MONITOR_PERIOD_MS = 300

# ====== 目標初期値 ======
INIT_TARGET = [350.0, 0.0, 300.0, 0.0, 180.0, 0.0]  # x,y,z, r,p,y


# ========== 数学ユーティリティ ==========
def fmt(n):
    return str(n) if isinstance(n, int) else f"{float(n):.6f}"

def rpy_deg_to_quat(r_deg, p_deg, y_deg):
    """ZYX順RPY[deg] → クォータニオン[ABB順: qx,qy,qz,qw]"""
    rx, ry, rz = map(math.radians, (r_deg, p_deg, y_deg))
    cx, sx = math.cos(rx/2), math.sin(rx/2)
    cy, sy = math.cos(ry/2), math.sin(ry/2)
    cz, sz = math.cos(rz/2), math.sin(rz/2)
    qw = cz*cy*cx + sz*sy*sx
    qx = cz*cy*sx - sz*sy*cx
    qy = cz*sy*cx + sz*cy*sx
    qz = sz*cy*cx - cz*sy*sx
    n = math.sqrt(qw*qw + qx*qx + qy*qy + qz*qz) or 1.0
    return [qx/n, qy/n, qz/n, qw/n]

def quat_to_rpy_deg(qx, qy, qz, qw):
    """クォータニオン → ZYX順RPY[deg]"""
    R11 = 1 - 2*(qy*qy + qz*qz)
    R21 = 2*(qx*qy + qz*qw)
    R31 = 2*(qx*qz - qy*qw)
    R32 = 2*(qy*qz + qx*qw)
    R33 = 1 - 2*(qx*qx + qy*qy)
    yaw  = math.degrees(math.atan2(R21, R11))
    pitch = math.degrees(math.asin(-R31))
    roll = math.degrees(math.atan2(R32, R33))
    return roll, pitch, yaw


# ========== open_abb クライアント ==========
class OpenABBRobotClient:
    def __init__(self, ip, port):
        self.ip = ip
        self.port = port
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
            try:
                self.sock.close()
            except Exception:
                pass
            self.sock = None

    def _send_cmd(self, parts, pause=0.18, expect=None):
        if not self.sock:
            raise RuntimeError("Not connected")
        msg = " ".join(fmt(p) for p in parts) + " #"
        with self.lock:
            self.sock.sendall(msg.encode("ascii"))
            time.sleep(pause)
            data = self.sock.recv(2048).decode("ascii", errors="ignore").strip()
        log.info(f"SEND: {msg.strip()}   RECV: {data}")
        toks = data.split()
        instr = int(toks[0]) if len(toks)>=1 and toks[0].lstrip("-").isdigit() else None
        ok    = int(toks[1]) if len(toks)>=2 and toks[1].lstrip("-").isdigit() else None
        if expect is not None and instr != expect:
            raise RuntimeError(f"ACK mismatch: expect {expect}, got {instr}, raw='{data}'")
        return instr, ok, toks

    @staticmethod
    def _parse_case3_tokens(toks):
        # 期待: "3 1 x y z qx qy qz qw"
        if len(toks) >= 9:
            return (float(toks[2]), float(toks[3]), float(toks[4]),
                    float(toks[5]), float(toks[6]), float(toks[7]), float(toks[8]))
        raise ValueError("CASE3 reply too short")

    def ping(self):
        return self._send_cmd([0], expect=0)

    def set_wobj(self, identity=True, wx=0, wy=0, wz=0, r=0, p=0, y=0):
        if identity:
            # 単位姿勢（open_abb仕様）: [qx,qy,qz,qw] = [1,0,0,0]
            wqx, wqy, wqz, wqw = 1.0, 0.0, 0.0, 0.0
            return self._send_cmd([7, 0.0, 0.0, 0.0, wqx, wqy, wqz, wqw], expect=7)
        else:
            wqx, wqy, wqz, wqw = rpy_deg_to_quat(r, p, y)
            return self._send_cmd([7, wx, wy, wz, wqx, wqy, wqz, wqw], expect=7)

    def set_tool(self, tx, ty, tz, r, p, y):
        tqx, tqy, tqz, tqw = rpy_deg_to_quat(r, p, y)
        return self._send_cmd([6, tx, ty, tz, tqx, tqy, tqz, tqw], expect=6)

    def set_speed_zone(self, v_tcp, v_ori, p_tcp, p_ori, z_ori):
        self._send_cmd([8, v_tcp, v_ori], expect=8)
        return self._send_cmd([9, 0, p_tcp, p_ori, z_ori], expect=9)

    def get_tcp(self):
        instr, ok, toks = self._send_cmd([3], expect=3)
        if ok == 1:
            x,y,z,qx,qy,qz,qw = self._parse_case3_tokens(toks)
            return True, (x,y,z,qx,qy,qz,qw)
        return False, None

    def movel_xyzrpy(self, x, y, z, r, p, yw):
        qx, qy, qz, qw = rpy_deg_to_quat(r, p, yw)
        instr, ok, _ = self._send_cmd([1, x, y, z, qx, qy, qz, qw], expect=1)
        return ok == 1


# ========== GUI ==========
class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("ABB IRB1200 TCP GUI (open_abb) - queue latest target")
        self.geometry("540x300")
        self.resizable(False, False)

        self.client = OpenABBRobotClient(ROBOT_IP, ROBOT_PORT)
        self.connected = False

        # 動作制御
        self.in_motion = False
        self.pending_target = None
        self.state_lock = threading.Lock()

        self._build_widgets()
        # 起動時に接続・初期化
        threading.Thread(target=self._init_robot, daemon=True).start()
        # モニタ更新開始
        self.after(MONITOR_PERIOD_MS, self._poll_tcp)

    def _build_widgets(self):
        pad = {'padx': 8, 'pady': 5}

        self.lbl_status = ttk.Label(self, text="Status: connecting...", foreground="blue")
        self.lbl_status.grid(row=0, column=0, columnspan=6, sticky="w", **pad)

        ttk.Label(self, text="Current TCP [mm/deg]").grid(row=1, column=0, columnspan=6, sticky="w", **pad)

        self.cur_vars = [tk.StringVar(value="---") for _ in range(6)]
        labels = ["x","y","z","roll","pitch","yaw"]
        for i, name in enumerate(labels):
            ttk.Label(self, text=f"{name}:").grid(row=2 + i//3, column=(i%3)*2+0, sticky="e", **pad)
            ttk.Label(self, textvariable=self.cur_vars[i], width=12).grid(row=2 + i//3, column=(i%3)*2+1, sticky="w", **pad)

        ttk.Label(self, text="Target XYZRPY [mm/deg]").grid(row=4, column=0, columnspan=6, sticky="w", **pad)

        self.ent_vars = []
        defaults = INIT_TARGET
        for i, name in enumerate(labels):
            ttk.Label(self, text=f"{name}:").grid(row=5 + i//3, column=(i%3)*2+0, sticky="e", **pad)
            var = tk.StringVar(value=str(defaults[i]))
            ent = ttk.Entry(self, textvariable=var, width=12)
            ent.grid(row=5 + i//3, column=(i%3)*2+1, sticky="w", **pad)
            self.ent_vars.append(var)

        self.btn_send = ttk.Button(self, text="Send MoveL", command=self._on_send)
        self.btn_send.grid(row=7, column=0, columnspan=6, **pad)

    # --- ロボット初期化 ---
    def _init_robot(self):
        try:
            self._set_status("connecting...", "blue")
            self.client.connect(timeout=5.0)
            self._set_status("connected", "green")

            self.client.ping()
            self.client.set_wobj(
                identity=IDENTITY_WOBJ,
                wx=WOBJ_WX, wy=WOBJ_WY, wz=WOBJ_WZ,
                r=WOBJ_R_DEG, p=WOBJ_P_DEG, y=WOBJ_Y_DEG
            )
            self.client.set_tool(TOOL_TX, TOOL_TY, TOOL_TZ, TOOL_R_DEG, TOOL_P_DEG, TOOL_Y_DEG)
            self.client.set_speed_zone(V_TCP, V_ORI, P_TCP, P_ORI, Z_ORI)
            self.connected = True
            log.info("Robot init done")
        except Exception as e:
            self._set_status(f"connect/init failed: {e}", "red")
            log.error("Init failed:\n" + traceback.format_exc())
            self.connected = False

    def _set_status(self, text, color):
        def _upd():
            self.lbl_status.config(text=f"Status: {text}", foreground=color)
        self.after(0, _upd)

    # --- モニタ周期（移動中も更新） ---
    def _poll_tcp(self):
        if self.connected:
            try:
                ok, tcp = self.client.get_tcp()
                if ok:
                    x,y,z,qx,qy,qz,qw = tcp
                    r,p,yw = quat_to_rpy_deg(qx,qy,qz,qw)
                    vals = [x,y,z,r,p,yw]
                    for i,v in enumerate(vals):
                        self.cur_vars[i].set(f"{v:.3f}")
                else:
                    self._set_status("get_tcp NG", "orange")
                    log.warning("get_tcp returned NG")
            except Exception as e:
                self._set_status(f"poll error: {e}", "red")
                log.error("Poll error:\n" + traceback.format_exc())
                threading.Thread(target=self._reconnect, daemon=True).start()
        self.after(MONITOR_PERIOD_MS, self._poll_tcp)

    def _reconnect(self):
        with self.state_lock:
            self.connected = False
        try:
            self._set_status("reconnecting...", "blue")
            self.client.close()
            self.client.connect(timeout=5.0)
            self.client.ping()
            self.client.set_wobj(
                identity=IDENTITY_WOBJ,
                wx=WOBJ_WX, wy=WOBJ_WY, wz=WOBJ_WZ,
                r=WOBJ_R_DEG, p=WOBJ_P_DEG, y=WOBJ_Y_DEG
            )
            self.client.set_tool(TOOL_TX, TOOL_TY, TOOL_TZ, TOOL_R_DEG, TOOL_P_DEG, TOOL_Y_DEG)
            self.client.set_speed_zone(V_TCP, V_ORI, P_TCP, P_ORI, Z_ORI)
            self._set_status("connected", "green")
            with self.state_lock:
                self.connected = True
            log.info("Reconnected")
        except Exception as e:
            self._set_status(f"reconnect failed: {e}", "red")
            log.error("Reconnect failed:\n" + traceback.format_exc())

    # --- 送信ボタン ---
    def _on_send(self):
        if not self.connected:
            messagebox.showerror("Error", "Not connected to robot.")
            return
        try:
            tgt = [float(v.get()) for v in self.ent_vars]  # x y z r p y
            x,y,z,r,p,yw = tgt
        except ValueError:
            messagebox.showerror("Error", "Invalid numeric input.")
            return

        with self.state_lock:
            if self.in_motion:
                # 最新1件だけ保持（上書き）
                self.pending_target = (x,y,z,r,p,yw)
                self._set_status("queued latest target", "orange")
                log.info(f"Target queued (latest only): {self.pending_target}")
                return
            # すぐ送る
            self.in_motion = True

        threading.Thread(target=self._do_movel_and_check_queue, args=((x,y,z,r,p,yw),), daemon=True).start()

    def _do_movel_and_check_queue(self, target):
        try:
            x,y,z,r,p,yw = target
            self._set_status("MoveL sending...", "blue")
            log.info(f"MoveL start to: {target}")
            ok = self.client.movel_xyzrpy(x,y,z,r,p,yw)
            if ok:
                self._set_status("MoveL ack OK", "green")
                log.info("MoveL ack OK")
            else:
                self._set_status("MoveL ack != 1", "orange")
                log.warning("MoveL ack != 1")

            # ちょい待って到達誤差確認
            time.sleep(0.5)
            ok_tcp, tcp = self.client.get_tcp()
            if ok_tcp:
                mx,my,mz,mqx,mqy,mqz,mqw = tcp
                pos_err = math.sqrt((mx-x)**2+(my-y)**2+(mz-z)**2)
                self._set_status(f"pos_err={pos_err:.2f} mm", "green")
                log.info(f"Reached? pos_err={pos_err:.3f} mm")
        except Exception as e:
            self._set_status(f"MoveL error: {e}", "red")
            log.error("MoveL error:\n" + traceback.format_exc())
        finally:
            # ここで最新キューを確認し，あれば直ちに送る
            next_target = None
            with self.state_lock:
                if self.pending_target is not None:
                    next_target = self.pending_target
                    self.pending_target = None
                    log.info(f"Dequeued latest target: {next_target}")
                else:
                    self.in_motion = False

            if next_target is not None:
                # 次の送信を続行（in_motionは継続trueのまま）
                self._do_movel_and_check_queue(next_target)
            else:
                # 完全にアイドルへ
                with self.state_lock:
                    self.in_motion = False


if __name__ == "__main__":
    app = App()
    app.mainloop()
