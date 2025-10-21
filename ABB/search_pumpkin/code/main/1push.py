# -*- coding: utf-8 -*-
# main/1push.py
# 目的:
#   ・GUIでXYZRPY(mm/deg)を指定し、main.pyと同様の“分割MoveL”で到達
#   ・起動直後から自動接続／切断時は自動再接続
#   ・push_plan.jsonは使わない／log保存なし
#   ・一時停止（再開トグル）ボタン
#   ・RobotClient.get_cartesian() が無い環境でも動作する互換シムを内蔵

import os
import sys
import math
import time
import threading
import tkinter as tk
from tkinter import ttk, messagebox

# --- パス解決 ---
BASE = os.path.dirname(os.path.abspath(__file__))              # .../push_manual/main
ROOT = os.path.abspath(os.path.join(BASE, os.pardir))          # .../push_manual
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

# 既存モジュール
from include.robot_client import RobotClient, RecoverableCommError
from include.math_utils import quat_to_rpy_deg
from info.config import (
    ROBOT_IP, ROBOT_PORT,
    DEFAULT_STEP_MM,
    DEFAULT_V_TCP, DEFAULT_V_ORI,
    DEFAULT_P_TCP, DEFAULT_P_ORI, DEFAULT_Z_ORI,
    DEFAULT_FINAL_FINE, DEFAULT_FINAL_P_TCP
)

# --- RobotClient.get_cartesian() 互換シム（CASE 3） ---
# 1) RobotClientにget_cartesianが未実装の場合のみ追加
# 2) 返却形式の差（[3, x..qw] or [x..qw]）に耐性
if not hasattr(RobotClient, "get_cartesian"):
    def _shim_get_cartesian(self):
        vals = self._send_and_recv([3], expect=3)  # CASE 3: Get Cartesian
        # 期待: x y z qx qy qz qw（計7要素）
        if not isinstance(vals, (list, tuple)):
            raise RuntimeError(f"get_cartesian: unexpected response type: {type(vals)}")
        arr = list(vals)
        # パターンA: [3, x, y, z, qx, qy, qz, qw]
        if len(arr) >= 8 and isinstance(arr[0], (int, float)) and abs(float(arr[0]) - 3.0) < 1e-6:
            arr = arr[1:8]
        # パターンB: [x, y, z, qx, qy, qz, qw]
        elif len(arr) == 7:
            pass
        # パターンC: それ以外 → 末尾7要素を採用
        elif len(arr) > 7:
            arr = arr[-7:]
        if len(arr) != 7:
            raise RuntimeError(f"get_cartesian: bad length {len(arr)}; vals={vals}")
        x, y, z, qx, qy, qz, qw = map(float, arr)
        rpy = quat_to_rpy_deg(qx, qy, qz, qw)  # degで返す
        return ((x, y, z), rpy)
    RobotClient.get_cartesian = _shim_get_cartesian

# ===== 差分送信キャッシュ =====
class ZoneSpeedCache:
    def __init__(self, client: RobotClient):
        self.c = client
        self._last_speed = None   # (v_tcp, v_ori)
        self._last_zone  = None   # (p_tcp, p_ori, z_ori)

    def set_speed_if_changed(self, v_tcp: float, v_ori: float):
        cur = (float(v_tcp), float(v_ori))
        if self._last_speed != cur:
            print(f"[DEBUG] set_speed {cur}")
            self.c.set_speed(v_tcp, v_ori)
            self._last_speed = cur

    def set_zone_if_changed(self, p_tcp: float, p_ori: float, z_ori: float):
        cur = (float(p_tcp), float(p_ori), float(z_ori))
        if self._last_zone != cur:
            print(f"[DEBUG] set_zone {cur}")
            self.c.set_zone(p_tcp, p_ori, z_ori)
            self._last_zone = cur

# ===== 角度補助（-180..180 の最短差で補間） =====
def _angle_diff_deg(a, b):
    """a->b の最短角度差（deg, 範囲 -180..180）"""
    d = (b - a + 180.0) % 360.0 - 180.0
    return d

def _angle_lerp_deg(a, b, t: float):
    """a から b に t (0..1) で線形補間（最短経路）"""
    return a + _angle_diff_deg(a, b) * t

# ===== GUI本体 =====
class OnePushApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("1Push - 分割MoveL (XYZRPY)")

        # Robot client とキャッシュ
        self.client = RobotClient(ROBOT_IP, ROBOT_PORT)
        self.zs = ZoneSpeedCache(self.client)

        # 変数
        self.var_conn = tk.StringVar(value="未接続")
        self.var_ip   = tk.StringVar(value=str(ROBOT_IP))
        self.var_port = tk.StringVar(value=str(ROBOT_PORT))

        # 目標姿勢 (mm/deg)
        self.var_x = tk.DoubleVar(value=500.0)
        self.var_y = tk.DoubleVar(value=0.0)
        self.var_z = tk.DoubleVar(value=300.0)
        self.var_r = tk.DoubleVar(value=180.0)
        self.var_p = tk.DoubleVar(value=0.0)
        self.var_yaw = tk.DoubleVar(value=0.0)

        # 分割・速度・ゾーン
        self.var_step_mm = tk.DoubleVar(value=float(DEFAULT_STEP_MM))
        self.var_v_tcp = tk.DoubleVar(value=float(DEFAULT_V_TCP))
        self.var_v_ori = tk.DoubleVar(value=float(DEFAULT_V_ORI))
        self.var_p_tcp = tk.DoubleVar(value=float(DEFAULT_P_TCP))
        self.var_p_ori = tk.DoubleVar(value=float(DEFAULT_P_ORI))
        self.var_z_ori = tk.DoubleVar(value=float(DEFAULT_Z_ORI))
        self.var_sleep_ms = tk.IntVar(value=10)  # 分割Move間のスロットル（ms）

        # 終点のみ微細ゾーン
        self.var_final_fine   = tk.BooleanVar(value=bool(DEFAULT_FINAL_FINE))
        self.var_final_p_tcp  = tk.DoubleVar(value=float(DEFAULT_FINAL_P_TCP))

        # 自動接続
        self._autoconnect_stop = threading.Event()
        self._autoconnect_thread = None
        self._reconnect_requested = True

        # 実行系
        self._move_thread = None
        self._pause_ev = threading.Event()  # True=一時停止中
        self._stop_ev  = threading.Event()  # 予約（将来の強制停止用）

        self._build_ui()
        self._start_auto_connect()

    # --- 接続状態（sock有無） ---
    def _client_connected(self) -> bool:
        return getattr(self.client, "sock", None) is not None

    # --- UI ---
    def _build_ui(self):
        frm_conn = ttk.LabelFrame(self, text="接続")
        frm_conn.grid(row=0, column=0, padx=10, pady=10, sticky="nwe")

        ttk.Label(frm_conn, text="IP:").grid(row=0, column=0, sticky="e")
        ent_ip = ttk.Entry(frm_conn, textvariable=self.var_ip, width=16)
        ent_ip.grid(row=0, column=1, padx=4)
        ttk.Label(frm_conn, text="Port:").grid(row=0, column=2, sticky="e")
        ent_port = ttk.Entry(frm_conn, textvariable=self.var_port, width=8)
        ent_port.grid(row=0, column=3, padx=4)

        self.btn_connect = ttk.Button(frm_conn, text="切断", command=self.on_toggle_connect)
        self.btn_connect.grid(row=0, column=4, padx=8)
        ttk.Label(frm_conn, textvariable=self.var_conn).grid(row=0, column=5, padx=8)

        ent_ip.bind("<FocusOut>", lambda e: self._mark_reconnect_needed())
        ent_port.bind("<FocusOut>", lambda e: self._mark_reconnect_needed())

        frm_pose = ttk.LabelFrame(self, text="目標姿勢 XYZRPY (mm/deg)")
        frm_pose.grid(row=1, column=0, padx=10, pady=6, sticky="we")

        for i,(lbl,var) in enumerate([
            ("X", self.var_x), ("Y", self.var_y), ("Z", self.var_z),
            ("Roll", self.var_r), ("Pitch", self.var_p), ("Yaw", self.var_yaw)
        ]):
            ttk.Label(frm_pose, text=f"{lbl}:").grid(row=0, column=2*i, sticky="e")
            ttk.Entry(frm_pose, textvariable=var, width=10).grid(row=0, column=2*i+1, padx=4)

        frm_param = ttk.LabelFrame(self, text="分割 / 速度・ゾーン / その他")
        frm_param.grid(row=2, column=0, padx=10, pady=6, sticky="we")

        # 分割
        ttk.Label(frm_param, text="step_mm:").grid(row=0, column=0, sticky="e")
        ttk.Entry(frm_param, textvariable=self.var_step_mm, width=8).grid(row=0, column=1, padx=4)
        ttk.Label(frm_param, text="sleep_ms:").grid(row=0, column=2, sticky="e")
        ttk.Entry(frm_param, textvariable=self.var_sleep_ms, width=8).grid(row=0, column=3, padx=4)

        # 速度
        ttk.Label(frm_param, text="v_tcp:").grid(row=0, column=4, sticky="e")
        ttk.Entry(frm_param, textvariable=self.var_v_tcp, width=8).grid(row=0, column=5, padx=4)
        ttk.Label(frm_param, text="v_ori:").grid(row=0, column=6, sticky="e")
        ttk.Entry(frm_param, textvariable=self.var_v_ori, width=8).grid(row=0, column=7, padx=4)

        # ゾーン
        ttk.Label(frm_param, text="p_tcp:").grid(row=1, column=0, sticky="e")
        ttk.Entry(frm_param, textvariable=self.var_p_tcp, width=8).grid(row=1, column=1, padx=4)
        ttk.Label(frm_param, text="p_ori:").grid(row=1, column=2, sticky="e")
        ttk.Entry(frm_param, textvariable=self.var_p_ori, width=8).grid(row=1, column=3, padx=4)
        ttk.Label(frm_param, text="z_ori:").grid(row=1, column=4, sticky="e")
        ttk.Entry(frm_param, textvariable=self.var_z_ori, width=8).grid(row=1, column=5, padx=4)

        ttk.Checkbutton(frm_param, text="終点のみ微細ゾーン", variable=self.var_final_fine).grid(row=1, column=6, sticky="w", padx=2)
        ttk.Label(frm_param, text="final p_tcp:").grid(row=1, column=7, sticky="e")
        ttk.Entry(frm_param, textvariable=self.var_final_p_tcp, width=8).grid(row=1, column=8, padx=4)

        frm_ops = ttk.Frame(self)
        frm_ops.grid(row=3, column=0, padx=10, pady=10, sticky="we")

        ttk.Button(frm_ops, text="現在姿勢を取得", command=self.on_get_current).grid(row=0, column=0, padx=4)
        ttk.Button(frm_ops, text="目標へMoveL（分割）", command=self.on_move_segmented).grid(row=0, column=1, padx=4)

        self.btn_pause = ttk.Button(frm_ops, text="一時停止", command=self.on_toggle_pause)
        self.btn_pause.grid(row=0, column=2, padx=8)

        self.txt_log = tk.Text(self, width=100, height=12)
        self.txt_log.grid(row=4, column=0, padx=10, pady=8, sticky="nsew")
        self.grid_rowconfigure(4, weight=1)
        self.grid_columnconfigure(0, weight=1)

    # --- 自動接続（2秒周期） ---
    def _start_auto_connect(self):
        if self._autoconnect_thread and self._autoconnect_thread.is_alive():
            return
        self._autoconnect_stop.clear()
        self._autoconnect_thread = threading.Thread(target=self._auto_connect_loop, daemon=True)
        self._autoconnect_thread.start()
        self._log("[INFO] 自動接続スレッド開始")

    def _auto_connect_loop(self):
        while not self._autoconnect_stop.is_set():
            try:
                need_reconnect = (not self._client_connected()) or getattr(self, "_reconnect_requested", True)
                if need_reconnect:
                    ip = self.var_ip.get().strip()
                    port = int(self.var_port.get())
                    self.client.host, self.client.port = ip, port

                    self._ui_set_status(f"接続試行中 {ip}:{port}", btn_text="切断")
                    try:
                        self.client.connect(timeout=5.0)
                        self._apply_zone_speed_safely()
                        self._ui_set_status(f"接続中 {ip}:{port}", btn_text="切断")
                        self._log(f"[INFO] Connected {ip}:{port}")
                        self._reconnect_requested = False
                    except Exception as e:
                        self._ui_set_status("未接続", btn_text="接続")
                        self._log(f"[WARN] 自動接続失敗: {e}")
            except Exception as e:
                self._log(f"[WARN] auto_connect_loop: {e}")
            finally:
                for _ in range(20):
                    if self._autoconnect_stop.is_set(): break
                    time.sleep(0.1)

    def _ui_set_status(self, status_text: str, btn_text: str):
        self.after(0, lambda: self.var_conn.set(status_text))
        self.after(0, lambda: self.btn_connect.config(text=btn_text))

    def _mark_reconnect_needed(self):
        self._reconnect_requested = True

    # --- 接続ボタン ---
    def on_toggle_connect(self):
        if self._client_connected():
            try:
                self.client.close()
                self._ui_set_status("未接続", btn_text="接続")
                self._log("[INFO] 切断しました。")
            except Exception as e:
                self._log(f"[WARN] 切断エラー: {e}")
            finally:
                self._mark_reconnect_needed()
        else:
            self._mark_reconnect_needed()

    # --- 速度・ゾーン ---
    def _apply_zone_speed_safely(self):
        v_tcp = float(self.var_v_tcp.get())
        v_ori = float(self.var_v_ori.get())
        p_tcp = float(self.var_p_tcp.get())
        p_ori = float(self.var_p_ori.get())
        z_ori = float(self.var_z_ori.get())
        self.zs.set_speed_if_changed(v_tcp, v_ori)
        self.zs.set_zone_if_changed(p_tcp, p_ori, z_ori)

    # --- 操作 ---
    def on_get_current(self):
        if not self._client_connected():
            messagebox.showwarning("未接続", "ロボットに接続してください。")
            self._mark_reconnect_needed()
            return
        try:
            xyz, rpy = self.client.get_cartesian()
            self._log("[GET] XYZ(mm)={}  RPY(deg)={}".format(
                [round(v, 3) for v in xyz],
                [round(v, 3) for v in rpy]
            ))
        except RecoverableCommError as e:
            self._log(f"[WARN] Recoverable: {e}")
            self._mark_reconnect_needed()
        except Exception as e:
            messagebox.showerror("取得エラー", str(e))
            self._log(f"[ERROR] 取得エラー: {e}")
            self._mark_reconnect_needed()

    def on_move_segmented(self):
        if not self._client_connected():
            messagebox.showwarning("未接続", "ロボットに接続してください。")
            self._mark_reconnect_needed()
            return

        # 二重起動防止
        if self._move_thread and self._move_thread.is_alive():
            self._log("[INFO] 既にMove中です。停止または完了を待ってください。")
            return

        # 入力
        target_xyz = (float(self.var_x.get()), float(self.var_y.get()), float(self.var_z.get()))
        target_rpy = (float(self.var_r.get()), float(self.var_p.get()), float(self.var_yaw.get()))
        step_mm    = max(0.1, float(self.var_step_mm.get()))
        sleep_ms   = max(0, int(self.var_sleep_ms.get()))

        # 速度・ゾーン差分送信
        try:
            self._apply_zone_speed_safely()
        except Exception as e:
            messagebox.showerror("送信エラー", str(e))
            self._log(f"[ERROR] set speed/zone: {e}")
            self._mark_reconnect_needed()
            return

        # 実行スレッド
        self._pause_ev.clear()
        self._stop_ev.clear()
        self._move_thread = threading.Thread(
            target=self._run_segmented_move, args=(target_xyz, target_rpy, step_mm, sleep_ms), daemon=True
        )
        self._move_thread.start()

    def _run_segmented_move(self, target_xyz, target_rpy, step_mm, sleep_ms):
        try:
            cur_xyz, cur_rpy = self.client.get_cartesian()
        except Exception as e:
            self._log(f"[ERROR] 現在姿勢取得に失敗: {e}")
            self._mark_reconnect_needed()
            return

        # 総距離とステップ数
        dx = target_xyz[0] - cur_xyz[0]
        dy = target_xyz[1] - cur_xyz[1]
        dz = target_xyz[2] - cur_xyz[2]
        dist = math.sqrt(dx*dx + dy*dy + dz*dz)
        steps = max(1, int(math.ceil(dist / step_mm)))

        self._log(f"[INFO] 分割Move: dist={dist:.1f}mm, steps={steps}, step_mm≈{(dist/steps):.2f}")

        try:
            for i in range(1, steps + 1):
                # 一時停止
                while self._pause_ev.is_set():
                    time.sleep(0.05)

                t = i / steps
                xyz = (cur_xyz[0] + dx * t,
                       cur_xyz[1] + dy * t,
                       cur_xyz[2] + dz * t)
                rpy = (_angle_lerp_deg(cur_rpy[0], target_rpy[0], t),
                       _angle_lerp_deg(cur_rpy[1], target_rpy[1], t),
                       _angle_lerp_deg(cur_rpy[2], target_rpy[2], t))

                # 終点だけ微細ゾーン
                if self.var_final_fine.get() and i == steps:
                    p_tcp_bak = float(self.var_p_tcp.get())
                    p_ori = float(self.var_p_ori.get())
                    z_ori = float(self.var_z_ori.get())
                    final_p_tcp = float(self.var_final_p_tcp.get())
                    self.zs.set_zone_if_changed(final_p_tcp, p_ori, z_ori)
                    self.client.moveL_ack(xyz[0], xyz[1], xyz[2], rpy)
                    self.zs.set_zone_if_changed(p_tcp_bak, p_ori, z_ori)
                else:
                    self.client.moveL_ack(xyz[0], xyz[1], xyz[2], rpy)

                if sleep_ms > 0:
                    time.sleep(sleep_ms / 1000.0)

            self._log(f"[OK] MoveL 完了: XYZ={target_xyz} RPY={target_rpy}")
        except RecoverableCommError as e:
            self._log(f"[WARN] Recoverable during segmented MoveL: {e}")
            self._mark_reconnect_needed()
        except Exception as e:
            messagebox.showerror("Moveエラー", str(e))
            self._log(f"[ERROR] MoveL失敗: {e}")
            self._mark_reconnect_needed()

    # --- 一時停止（再開トグル） ---
    def on_toggle_pause(self):
        if not (self._move_thread and self._move_thread.is_alive()):
            self._log("[INFO] 現在Moveは動作していません。")
            return
        if self._pause_ev.is_set():
            self._pause_ev.clear()
            self.btn_pause.config(text="一時停止")
            self._log("[INFO] 再開しました。")
        else:
            self._pause_ev.set()
            self.btn_pause.config(text="再開")
            self._log("[INFO] 一時停止しました。")

    # --- 共通 ---
    def _log(self, s: str):
        print(s)
        self.txt_log.insert("end", s + "\n")
        self.txt_log.see("end")

    def destroy(self):
        try:
            self._autoconnect_stop.set()
            if self.client and getattr(self.client, "sock", None) is not None:
                self.client.close()
        finally:
            super().destroy()

# エントリポイント
if __name__ == "__main__":
    app = OnePushApp()
    app.geometry("1000x600")
    app.mainloop()
