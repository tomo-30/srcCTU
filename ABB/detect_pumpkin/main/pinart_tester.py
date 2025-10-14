# -*- coding: utf-8 -*-
# main/pinart_tester.py
# 目的:
#  - Robot/Arduino と自動接続
#  - 目的地(x,y,z,roll,pitch,yaw)をGUI入力→MoveLで移動できる
#  - matplotlib 3D で 手先位置(赤▲) と ピン先端(青×/青○) をリアルタイム表示（単一グラフ）
#  - info/pinart_info.json を用いて，手先姿勢(クォータニオン)→回転行列→ピン先端を計算
#
# 依存:
#  - include/robot_client.py（RobotClient, RecoverableCommError）
#  - info/pinart_info.json（提供データ）
#  - pip install pyserial matplotlib

import os
import sys
import json
import time
import math
import threading
import tkinter as tk
from tkinter import ttk, messagebox

# ====== プロジェクトパス ======
BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))  # project/
if BASE_DIR not in sys.path:
    sys.path.append(BASE_DIR)

from include.robot_client import RobotClient, RecoverableCommError

# ====== シリアル依存 ======
try:
    import serial
    import serial.tools.list_ports as list_ports
except Exception:
    serial = None
    list_ports = None

# ====== matplotlib（Tk埋め込み・3D） ======
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

# ====== 既定設定 ======
ROBOT_IP, ROBOT_PORT = "192.168.125.1", 5000
DEFAULT_BAUD = 115200

DEFAULT_V_TCP = 40
DEFAULT_V_ORI = 80
DEFAULT_P_TCP = 1.5
DEFAULT_P_ORI = 8
DEFAULT_Z_ORI = 8

# 3D表示範囲（mm）
DEFAULT_X_MIN, DEFAULT_X_MAX = 200, 600
DEFAULT_Y_MIN, DEFAULT_Y_MAX = -200, 200
DEFAULT_Z_MIN, DEFAULT_Z_MAX = 400, 900

# info/pinart_info.json
PINART_INFO_PATH = os.path.join(BASE_DIR, "info", "pinart_info.json")

# ピンの伸び方向（手先座標系 -Z を +1 とした符号。+1= -Z方向に伸びる仮定，-1= +Z方向に伸びる）
PIN_DIR_SIGN = +1


# ---------------- ユーティリティ（姿勢変換） ----------------
def rpy_deg_to_rot(r, p, y):
    """roll, pitch, yaw [deg] -> 3x3 回転行列（右手系 ZYX）"""
    rr, pp, yy = map(math.radians, (r, p, y))
    cr, sr = math.cos(rr), math.sin(rr)
    cp, sp = math.cos(pp), math.sin(pp)
    cy, sy = math.cos(yy), math.sin(yy)
    Rz = ((cy, -sy, 0),
          (sy,  cy, 0),
          ( 0,   0, 1))
    Ry = ((cp, 0, sp),
          ( 0, 1,  0),
          (-sp,0, cp))
    Rx = ((1, 0,  0),
          (0, cr,-sr),
          (0, sr, cr))
    def matmul(A,B):
        return tuple(tuple(sum(A[i][k]*B[k][j] for k in range(3)) for j in range(3)) for i in range(3))
    return matmul(matmul(Rz, Ry), Rx)

def apply_rot(R, v):
    x = R[0][0]*v[0] + R[0][1]*v[1] + R[0][2]*v[2]
    y = R[1][0]*v[0] + R[1][1]*v[1] + R[1][2]*v[2]
    z = R[2][0]*v[0] + R[2][1]*v[1] + R[2][2]*v[2]
    return (x,y,z)

def quat_to_rot(qx, qy, qz, qw):
    """クォータニオン→3x3回転行列（ローカル→ワールド）"""
    # 正規化
    n = math.sqrt(qw*qw + qx*qx + qy*qy + qz*qz) or 1.0
    qw, qx, qy, qz = qw/n, qx/n, qy/n, qz/n
    xx, yy, zz = qx*qx, qy*qy, qz*qz
    xy, xz, yz = qx*qy, qx*qz, qy*qz
    wx, wy, wz = qw*qx, qw*qy, qw*qz
    # 標準の回転行列
    return (
        (1 - 2*(yy + zz),     2*(xy - wz),       2*(xz + wy)),
        (    2*(xy + wz), 1 - 2*(xx + zz),       2*(yz - wx)),
        (    2*(xz - wy),     2*(yz + wx),   1 - 2*(xx + yy)),
    )

# ------- クォータニオン→RPY（deg）：GUI表示用（MoveLの入力補助） -------
def quat_to_rpy_deg(qx, qy, qz, qw):
    n = math.sqrt(qw*qw + qx*qx + qy*qy + qz*qz) or 1.0
    qw, qx, qy, qz = qw/n, qx/n, qy/n, qz/n
    roll  = math.degrees(math.atan2(2*(qw*qx + qy*qz), 1 - 2*(qx*qx + qy*qy)))
    pitch = math.degrees(math.asin(max(-1.0, min(1.0, 2*(qw*qy - qz*qx)))))
    yaw   = math.degrees(math.atan2(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz)))
    return (roll, pitch, yaw)


# ---------------- Arduino シリアル監視（自動接続） ----------------
class ArduinoSerialMonitor:
    def __init__(self):
        self._ser = None
        self._thread = None
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._curr = [-1]*9
        self._ts = 0.0
        self._status = "serial: disconnected"
        self._baud = DEFAULT_BAUD

    def is_connected(self) -> bool:
        return (self._ser is not None) and getattr(self._ser, "is_open", False)

    def status(self) -> str:
        return self._status

    def list_ports(self):
        if list_ports is None:
            return []
        return list(list_ports.comports())

    def try_autoconnect_once(self):
        if self.is_connected() or serial is None:
            return
        port = None
        for p in self.list_ports():
            desc = (p.description or "").lower()
            if "arduino" in desc or "usb serial" in desc or "wch" in desc:
                port = p.device
                break
        if not port:
            lst = self.list_ports()
            if lst:
                port = lst[0].device
        if not port:
            self._status = "serial: no port found"
            return
        try:
            self._connect(port, self._baud)
        except Exception as e:
            self._status = f"serial: error ({e})"

    def _connect(self, port, baud):
        self._ser = serial.Serial(
            port=port,
            baudrate=baud,
            timeout=1.0,
            write_timeout=1.0,
            rtscts=False,
            dsrdtr=False
        )
        time.sleep(1.5)
        try:
            self._ser.reset_input_buffer()
        except:
            pass
        self._stop.clear()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        self._status = f"serial: connected ({port} @ {baud})"

    def disconnect(self):
        self._stop.set()
        if self._thread and self._thread.is_alive():
            try: self._thread.join(timeout=0.5)
            except: pass
        self._thread = None
        if self._ser:
            try: self._ser.close()
            except: pass
        self._ser = None
        self._status = "serial: disconnected"

    def get_latest(self):
        with self._lock:
            return list(self._curr), self._ts

    def _loop(self):
        while not self._stop.is_set() and self._ser is not None:
            try:
                line = self._ser.readline()
                if not line:
                    continue
                s = line.decode("utf-8", errors="ignore").strip()
                if not s:
                    continue
                vals = [-1]*9
                try:
                    for part in s.split(","):
                        part = part.strip()
                        if not part or ":" not in part:
                            continue
                        k, v = part.split(":", 1)
                        if not k.isdigit():
                            continue
                        idx = int(k) - 1
                        if 0 <= idx < 9:
                            vals[idx] = int(v.strip())
                except Exception:
                    pass
                with self._lock:
                    self._curr = vals
                    self._ts = time.time()
            except Exception:
                time.sleep(0.02)
                continue


# ---------------- GUI アプリ ----------------
class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("PinArt Tester 3D (Auto-connect Robot & Arduino)")
        self.geometry("1120x900")

        # Robot
        self.client = None
        self.robot_connected = False

        # Arduino
        self.arduino = ArduinoSerialMonitor()

        # PinArt Info
        self.pin_info = self._load_pin_info(PINART_INFO_PATH)

        # ---- 上段：接続・表示範囲・速度ゾーン・目的地入力 ----
        self._build_top_panel()

        # ---- 中段：3D表示 ----
        self._build_plot3d()

        # 自動接続メンテ開始
        self.after(200, self._auto_connect_maint)
        # 表示更新
        self.after(150, self._periodic_update)

    # ====== GUI ======
    def _build_top_panel(self):
        frm = ttk.LabelFrame(self, text="接続・表示範囲・速度ゾーン・目的地MoveL（接続は自動）")
        frm.pack(padx=10, pady=8, fill="x")

        # ステータス
        self.var_robot_status = tk.StringVar(value="robot: connecting...")
        self.var_serial_status = tk.StringVar(value="serial: connecting...")
        ttk.Label(frm, textvariable=self.var_robot_status, foreground="#333").grid(row=0, column=0, columnspan=8, sticky="w", padx=6)
        ttk.Label(frm, textvariable=self.var_serial_status, foreground="#333").grid(row=1, column=0, columnspan=8, sticky="w", padx=6)

        # 表示範囲
        ttk.Label(frm, text="Xmin").grid(row=0, column=8, sticky="e"); self.var_xmin = tk.StringVar(value=str(DEFAULT_X_MIN))
        ttk.Entry(frm, textvariable=self.var_xmin, width=7).grid(row=0, column=9, sticky="w")
        ttk.Label(frm, text="Xmax").grid(row=0, column=10, sticky="e"); self.var_xmax = tk.StringVar(value=str(DEFAULT_X_MAX))
        ttk.Entry(frm, textvariable=self.var_xmax, width=7).grid(row=0, column=11, sticky="w")
        ttk.Label(frm, text="Ymin").grid(row=0, column=12, sticky="e"); self.var_ymin = tk.StringVar(value=str(DEFAULT_Y_MIN))
        ttk.Entry(frm, textvariable=self.var_ymin, width=7).grid(row=0, column=13, sticky="w")
        ttk.Label(frm, text="Ymax").grid(row=0, column=14, sticky="e"); self.var_ymax = tk.StringVar(value=str(DEFAULT_Y_MAX))
        ttk.Entry(frm, textvariable=self.var_ymax, width=7).grid(row=0, column=15, sticky="w")
        ttk.Label(frm, text="Zmin").grid(row=1, column=8, sticky="e"); self.var_zmin = tk.StringVar(value=str(DEFAULT_Z_MIN))
        ttk.Entry(frm, textvariable=self.var_zmin, width=7).grid(row=1, column=9, sticky="w")
        ttk.Label(frm, text="Zmax").grid(row=1, column=10, sticky="e"); self.var_zmax = tk.StringVar(value=str(DEFAULT_Z_MAX))
        ttk.Entry(frm, textvariable=self.var_zmax, width=7).grid(row=1, column=11, sticky="w")
        ttk.Button(frm, text="範囲適用", command=self._apply_plot_ranges).grid(row=1, column=12, padx=6, sticky="w")

        # 速度/ゾーン（自動接続後に適用）
        ttk.Label(frm, text="V_TCP").grid(row=0, column=16, sticky="e"); self.var_vtcp = tk.StringVar(value=str(DEFAULT_V_TCP))
        ttk.Entry(frm, textvariable=self.var_vtcp, width=7).grid(row=0, column=17, sticky="w")
        ttk.Label(frm, text="V_ORI").grid(row=0, column=18, sticky="e"); self.var_vori = tk.StringVar(value=str(DEFAULT_V_ORI))
        ttk.Entry(frm, textvariable=self.var_vori, width=7).grid(row=0, column=19, sticky="w")
        ttk.Label(frm, text="P_TCP").grid(row=1, column=16, sticky="e"); self.var_ptcp = tk.StringVar(value=str(DEFAULT_P_TCP))
        ttk.Entry(frm, textvariable=self.var_ptcp, width=7).grid(row=1, column=17, sticky="w")
        ttk.Label(frm, text="P_ORI").grid(row=1, column=18, sticky="e"); self.var_pori = tk.StringVar(value=str(DEFAULT_P_ORI))
        ttk.Entry(frm, textvariable=self.var_pori, width=7).grid(row=1, column=19, sticky="w")
        ttk.Label(frm, text="Z_ORI").grid(row=1, column=20, sticky="e"); self.var_zori = tk.StringVar(value=str(DEFAULT_Z_ORI))
        ttk.Entry(frm, textvariable=self.var_zori, width=7).grid(row=1, column=21, sticky="w")
        ttk.Button(frm, text="適用", command=self._apply_speed_zone).grid(row=1, column=22, padx=6, sticky="w")

        # 目的地入力（MoveL）
        ttk.Separator(frm, orient="horizontal").grid(row=2, column=0, columnspan=23, sticky="ew", pady=6)
        labels = ["x(mm)","y(mm)","z(mm)","roll(deg)","pitch(deg)","yaw(deg)"]
        self.var_tx = tk.StringVar(value="400.0")
        self.var_ty = tk.StringVar(value="0.0")
        self.var_tz = tk.StringVar(value="300.0")
        self.var_tr = tk.StringVar(value="0.0")
        self.var_tp = tk.StringVar(value="1.0")
        self.var_tyaw = tk.StringVar(value="0.0")
        tvars = [self.var_tx, self.var_ty, self.var_tz, self.var_tr, self.var_tp, self.var_tyaw]
        for i,(lab,var) in enumerate(zip(labels, tvars)):
            ttk.Label(frm, text=lab).grid(row=3, column=i*2, sticky="e", padx=3)
            ttk.Entry(frm, textvariable=var, width=10).grid(row=3, column=i*2+1, sticky="w")
        ttk.Button(frm, text="MoveL へ移動", command=self._on_move_to_pose).grid(row=3, column=12, padx=8, sticky="w")
        ttk.Button(frm, text="現在姿勢を取得→入力欄へ反映", command=self._on_fetch_pose_into_fields).grid(row=3, column=13, columnspan=4, padx=8, sticky="w")

    def _build_plot3d(self):
        frm = ttk.LabelFrame(self, text="3D リアルタイム表示（手先＝赤▲，ピン先端＝青×/青○）")
        frm.pack(padx=10, pady=8, fill="both", expand=True)

        self.fig = Figure(figsize=(7.8, 6.4), dpi=100)
        self.ax3 = self.fig.add_subplot(1,1,1, projection="3d")
        self.ax3.set_xlabel("X [mm]"); self.ax3.set_ylabel("Y [mm]"); self.ax3.set_zlabel("Z [mm]")
        self.ax3.set_xlim(DEFAULT_X_MIN, DEFAULT_X_MAX)
        self.ax3.set_ylim(DEFAULT_Y_MIN, DEFAULT_Y_MAX)
        self.ax3.set_zlim(DEFAULT_Z_MIN, DEFAULT_Z_MAX)
        self.ax3.view_init(elev=25, azim=-60)
        self.ax3.grid(True, alpha=0.3)

        # 手先位置（赤▲）
        self._ee_plot = self.ax3.plot([], [], [], marker="^", markersize=10, linestyle="None", color="red")[0]
        # ピン先端：-1以外（縮み：青○）
        self._pin_o = self.ax3.plot([], [], [], "o", color="blue", markersize=7, linestyle="None")[0]
        # ピン先端：-1（伸びきり：青×）
        self._pin_x = self.ax3.plot([], [], [], "x", color="blue", markersize=7, linestyle="None")[0]

        canvas = FigureCanvasTkAgg(self.fig, master=frm)
        canvas.draw()
        canvas.get_tk_widget().pack(fill="both", expand=True)
        self._canvas = canvas

    # ====== Pin Info 読み込み ======
    def _load_pin_info(self, path):
        try:
            with open(path, "r", encoding="utf-8") as f:
                info = json.load(f)
        except Exception as e:
            messagebox.showerror("pinart_info.json 読込エラー", f"{e}")
            info = {
                "num_pins": 9,
                "pin_diameter_mm": 10.0,
                "pin_length_mm": 30.0,
                "pin_bases_wrist_xyz_mm": [[0,0,50.0]]*9
            }
        return info

    # ====== 自動接続メンテ ======
    def _auto_connect_maint(self):
        # Robot
        if not self.robot_connected:
            try:
                self.client = RobotClient(ROBOT_IP, ROBOT_PORT)
                self.client.connect(timeout=5.0)
                self.client.ping()
                self.robot_connected = True
                self.var_robot_status.set(f"robot: connected ({ROBOT_IP}:{ROBOT_PORT})")
                self._apply_speed_zone(silent=True)
            except Exception as e:
                self.client = None
                self.robot_connected = False
                self.var_robot_status.set(f"robot: retrying... ({e})")

        # Arduino
        if not self.arduino.is_connected():
            self.arduino.try_autoconnect_once()
            self.var_serial_status.set(self.arduino.status())

        self.after(1500, self._auto_connect_maint)

    # ====== 表示更新 ======
    def _periodic_update(self):
        ee_pos = None
        R = None
        if self.robot_connected and self.client:
            try:
                x, y, z, qx, qy, qz, qw = self.client.get_cart()
                # ★修正点：クォータニオン→回転行列を直接使用
                R = quat_to_rot(qx, qy, qz, qw)
                ee_pos = (x, y, z)
                self._ee_plot.set_data([x], [y]); self._ee_plot.set_3d_properties([z])
            except Exception:
                pass

        pins, _ = self.arduino.get_latest()

        bases_wrist = self.pin_info.get("pin_bases_wrist_xyz_mm", [])
        pin_len_full = float(self.pin_info.get("pin_length_mm", 30.0))

        xs_o, ys_o, zs_o = [], [], []
        xs_x, ys_x, zs_x = [], [], []

        if ee_pos is not None and R is not None and bases_wrist:
            # 手先の -Z（単位）
            ez_wrist = (0.0, 0.0, -1.0 * PIN_DIR_SIGN)
            ez_world = apply_rot(R, ez_wrist)

            for i, base in enumerate(bases_wrist):
                if not isinstance(base, (list, tuple)) or len(base) < 3:
                    continue
                bx_w, by_w, bz_w = base
                b_world = apply_rot(R, (bx_w, by_w, bz_w))
                bwx = ee_pos[0] + b_world[0]
                bwy = ee_pos[1] + b_world[1]
                bwz = ee_pos[2] + b_world[2]

                val = pins[i] if i < len(pins) else -1
                length = pin_len_full if (val == -1) else max(0.0, float(val))
                tip = (bwx + ez_world[0]*length,
                       bwy + ez_world[1]*length,
                       bwz + ez_world[2]*length)
                if val == -1:
                    xs_x.append(tip[0]); ys_x.append(tip[1]); zs_x.append(tip[2])
                else:
                    xs_o.append(tip[0]); ys_o.append(tip[1]); zs_o.append(tip[2])

        self._pin_o.set_data(xs_o, ys_o); self._pin_o.set_3d_properties(zs_o)
        self._pin_x.set_data(xs_x, ys_x); self._pin_x.set_3d_properties(zs_x)

        self.var_serial_status.set(self.arduino.status())
        self.var_robot_status.set("robot: connected" if self.robot_connected else "robot: retrying...")

        self._canvas.draw_idle()
        self.after(150, self._periodic_update)

    # ====== 操作系 ======
    def _apply_plot_ranges(self):
        try:
            xmin = float(self.var_xmin.get()); xmax = float(self.var_xmax.get())
            ymin = float(self.var_ymin.get()); ymax = float(self.var_ymax.get())
            zmin = float(self.var_zmin.get()); zmax = float(self.var_zmax.get())
        except:
            messagebox.showerror("範囲", "数値が不正です。")
            return
        if not (xmin < xmax and ymin < ymax and zmin < zmax):
            messagebox.showerror("範囲", "最小 < 最大 となるように設定してください。")
            return
        self.ax3.set_xlim(xmin, xmax)
        self.ax3.set_ylim(ymin, ymax)
        self.ax3.set_zlim(zmin, zmax)
        self._canvas.draw_idle()

    def _apply_speed_zone(self, silent=False):
        if not self.robot_connected or not self.client:
            return
        try:
            v_tcp = float(self.var_vtcp.get()); v_ori = float(self.var_vori.get())
            p_tcp = float(self.var_ptcp.get()); p_ori = float(self.var_pori.get()); z_ori = float(self.var_zori.get())
            self.client.set_speed(v_tcp, v_ori)
            self.client.set_zone(p_tcp, p_ori, z_ori)
        except Exception as e:
            if not silent:
                messagebox.showerror("設定エラー", f"{e}")

    def _on_move_to_pose(self):
        if not (self.robot_connected and self.client):
            messagebox.showwarning("MoveL", "ロボット未接続です。")
            return
        try:
            x = float(self.var_tx.get()); y = float(self.var_ty.get()); z = float(self.var_tz.get())
            r = float(self.var_tr.get()); p = float(self.var_tp.get()); yw = float(self.var_tyaw.get())
        except:
            messagebox.showerror("入力", "姿勢の数値が不正です。")
            return
        try:
            self.client.moveL_ack(x, y, z, (r, p, yw))
        except RecoverableCommError:
            try:
                time.sleep(0.2)
                self.client.moveL_ack(x, y, z, (r, p, yw))
            except Exception as e:
                messagebox.showerror("MoveL失敗", f"{e}")
        except Exception as e:
            messagebox.showerror("MoveL失敗", f"{e}")

    def _on_fetch_pose_into_fields(self):
        if not (self.robot_connected and self.client):
            messagebox.showwarning("姿勢取得", "ロボット未接続です。")
            return
        try:
            x, y, z, qx, qy, qz, qw = self.client.get_cart()
            r, p, yw = quat_to_rpy_deg(qx, qy, qz, qw)
            self.var_tx.set(f"{x:.2f}"); self.var_ty.set(f"{y:.2f}"); self.var_tz.set(f"{z:.2f}")
            self.var_tr.set(f"{r:.2f}"); self.var_tp.set(f"{p:.2f}"); self.var_tyaw.set(f"{yw:.2f}")
        except Exception as e:
            messagebox.showerror("姿勢取得エラー", f"{e}")

    # ====== 終了処理 ======
    def destroy(self):
        try:
            if self.client: self.client.close()
        except: pass
        try:
            self.arduino.disconnect()
        except: pass
        super().destroy()


if __name__ == "__main__":
    app = App()
    app.mainloop()
