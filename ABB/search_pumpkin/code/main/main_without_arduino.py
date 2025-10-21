# -*- coding: utf-8 -*-
# main/main.py  (sync pins+pose logging, hold-until-saved, auto-resume on comm error)

import os, sys, json, math, time, threading
import tkinter as tk
from tkinter import ttk, messagebox, filedialog

# ==== パス設定 ====
BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))  # project/
if BASE_DIR not in sys.path:
    sys.path.append(BASE_DIR)

from include.robot_client import RobotClient, RecoverableCommError
from include.utils_json import make_log_filename, write_log_entry, timestamp_str

# ==== 接続先 ====
ROBOT_IP, ROBOT_PORT = "192.168.125.1", 5000

# ==== 既定パラメータ ====
DEFAULT_STEP_MM = 4.0
DEFAULT_V_TCP   = 40
DEFAULT_V_ORI   = 80
DEFAULT_P_TCP   = 1.5
DEFAULT_P_ORI   = 8
DEFAULT_Z_ORI   = 8
DEFAULT_LOOKAHEAD = 3
DEFAULT_FINAL_FINE = True
DEFAULT_FINAL_P_TCP = 0.5

# ==== Arduino関連 ====
SIM_PERIOD_SEC = 0.10     # 100ms
OVERRIDE_BURST = 5        # （残置）短期上書きに使いたい場合用

# ==== ピン閾値（GUIで変更可能）====
DEFAULT_THRESH_ANY_MM = 5.0   # ② いずれかのピンが <= この値
DEFAULT_THRESH_ALL_MM = 10.0  # ③ すべてのピンが <= この値

# ==== ①②③時の停止時間（GUIで変更可能）====
DEFAULT_PAUSE_AFTER_EVENT_SEC = 1.0

# ==== ファイル ====
PUSH_PLAN_PATH = os.path.join(BASE_DIR, "info", "push_plan.json")
LOG_DIR        = os.path.join(BASE_DIR, "log")


# ---- 角度変換 ----
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

def quat_to_rpy_deg(qx, qy, qz, qw):
    n = math.sqrt(qw*qw + qx*qx + qy*qy + qz*qz) or 1.0
    qw, qx, qy, qz = qw/n, qx/n, qy/n, qz/n
    roll  = math.degrees(math.atan2(2*(qw*qx + qy*qz), 1 - 2*(qx*qx + qy*qy)))
    pitch = math.degrees(math.asin(max(-1.0, min(1.0, 2*(qw*qy - qz*qx)))))
    yaw   = math.degrees(math.atan2(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz)))
    return (roll, pitch, yaw)


# ---- 経路分割 ----
def make_via_points_by_step(curr_xyz, target_xyz, step_mm: float):
    cx, cy, cz = curr_xyz
    tx, ty, tz = target_xyz
    dx, dy, dz = tx - cx, ty - cy, tz - cz
    dist = math.sqrt(dx*dx + dy*dy + dz*dz)
    if dist <= 1e-9:
        return [target_xyz]
    n_seg = max(1, int(dist // max(step_mm, 1e-6)))
    via = []
    for i in range(1, n_seg + 1):
        s = i / n_seg
        via.append((cx + dx*s, cy + dy*s, cz + dz*s))
    via[-1] = (tx, ty, tz)
    return via


# ---- 計画読み込み ----
def load_plan_sequence(json_path):
    with open(json_path, "r", encoding="utf-8") as f:
        data = json.load(f)
    plan = data.get("plan", [])
    plan_sorted = sorted(plan, key=lambda d: d.get("id", 0))
    seq = []  # [(label, (x,y,z,r,p,y), id), ...]
    for item in plan_sorted:
        _id = item.get("id")
        A = item.get("A"); B = item.get("B")
        if not (A and B) or len(A) < 6 or len(B) < 6:
            raise ValueError(f"id={_id} のA/Bが不正（[x,y,z,roll,pitch,yaw]の6要素）")
        Ax,Ay,Az,Ar,Ap,Ayaw = map(float, A)
        Bx,By,Bz,Br,Bp,Byaw = map(float, B)
        seq.append((f"A{_id}", (Ax,Ay,Az,Ar,Ap,Ayaw), _id))
        seq.append((f"B{_id}", (Bx,By,Bz,Br,Bp,Byaw), _id))
        seq.append((f"A{_id}", (Ax,Ay,Az,Ar,Ap,Ayaw), _id))
    return seq


# ==== Arduinoシミュレータ（保持モード対応） ====
class ArduinoSim:
    """
    通常は100msごとに全-1．
    GUIで「送信」→ holdに入り，**保存完了まで**その値を出し続ける．
    （必要なら従来の5回バーストも残置）
    """
    def __init__(self):
        self._lock = threading.Lock()
        self._curr = [-1]*9
        self._override = None      # バースト用
        self._burst_left = 0
        self._hold_vals = None     # 保持用（list[9]）；Noneで非保持
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._last_update_ts = 0.0

    def start(self):
        self._thread.start()

    def stop(self):
        self._stop.set()
        self._thread.join(timeout=0.5)

    def set_override_5times(self, vals9):
        if not (isinstance(vals9, (list, tuple)) and len(vals9) == 9):
            return
        with self._lock:
            self._override = list(map(int, vals9))
            self._burst_left = OVERRIDE_BURST

    def hold_until_release(self, vals9):
        if not (isinstance(vals9, (list, tuple)) and len(vals9) == 9):
            return
        with self._lock:
            self._hold_vals = list(map(int, vals9))

    def release_hold(self):
        with self._lock:
            self._hold_vals = None

    def get_latest(self):
        with self._lock:
            return list(self._curr), self._last_update_ts

    def _loop(self):
        while not self._stop.is_set():
            with self._lock:
                if self._hold_vals is not None:
                    out = list(self._hold_vals)
                elif self._override is not None and self._burst_left > 0:
                    out = list(self._override)
                    self._burst_left -= 1
                    if self._burst_left == 0:
                        self._override = None
                else:
                    out = [-1]*9
                self._curr = out
                self._last_update_ts = time.time()
            time.sleep(SIM_PERIOD_SEC)


# ==== GUI/アプリ本体 ====
class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("open_abb plan runner + ArduinoSim")
        self.geometry("900x740")

        # ---- Robot client ----
        self.client = RobotClient(ROBOT_IP, ROBOT_PORT)

        # ---- 状態 ----
        self.sequence = []
        self.seq_idx = 0
        self.pause_flag = threading.Event()
        self.skip_flag  = threading.Event()
        self.motion_thread = None
        self.log_file_path = None

        # ---- Arduino Sim ----
        self.arduino = ArduinoSim()
        self.arduino.start()

        # ---- 上段: ログ保存モード ----
        self._build_log_mode_frame()

        # ---- 中段: 現在の目的地表示 ----
        self._build_current_target_frame()

        # ---- パラメータ（速度・ゾーン・分割・閾値・停止時間）----
        self._build_params_frame()

        # ---- ArduinoシミュレーションGUI ----
        self._build_arduino_frame()

        # ---- 操作ボタン ----
        self._build_buttons()

        # ---- 自動接続＆初期化 ----
        self.after(100, self._auto_connect_and_init)

    # === GUI構築 ===
    def _build_log_mode_frame(self):
        frm = ttk.LabelFrame(self, text="ログ保存モード（開始前に選択）")
        frm.pack(padx=10, pady=6, fill="x")
        self.var_log_mode = tk.StringVar(value="new")
        self.var_log_path = tk.StringVar(value="")
        rb_new  = ttk.Radiobutton(frm, text="新規保存（日時ファイル名）", value="new",   variable=self.var_log_mode, command=self._on_log_mode_change)
        rb_app  = ttk.Radiobutton(frm, text="追記（既存を参照）",       value="append", variable=self.var_log_mode, command=self._on_log_mode_change)
        rb_none = ttk.Radiobutton(frm, text="保存しない",               value="none",  variable=self.var_log_mode, command=self._on_log_mode_change)
        rb_new.grid(row=0, column=0, sticky="w", padx=6, pady=4)
        rb_app.grid(row=0, column=1, sticky="w", padx=6, pady=4)
        rb_none.grid(row=0, column=2, sticky="w", padx=6, pady=4)
        self.btn_choose_log = ttk.Button(frm, text="参照…", command=self._choose_log_file, state="disabled")
        self.btn_choose_log.grid(row=0, column=3, sticky="w", padx=6)
        ttk.Label(frm, textvariable=self.var_log_path, foreground="#555").grid(row=1, column=0, columnspan=4, sticky="w", padx=8, pady=2)

    def _build_current_target_frame(self):
        top = ttk.LabelFrame(self, text="現在の目的地")
        top.pack(padx=10, pady=8, fill="x")
        self.var_curr_label = tk.StringVar(value="-")
        self.var_curr_xyzrpy = tk.StringVar(value="x= -, y= -, z= -, roll= -, pitch= -, yaw= -")
        ttk.Label(top, textvariable=self.var_curr_label, font=("", 14, "bold"), width=10)\
            .grid(row=0, column=0, sticky="w", padx=6, pady=6)
        ttk.Label(top, textvariable=self.var_curr_xyzrpy, font=("", 12))\
            .grid(row=0, column=1, sticky="w", padx=6, pady=6)

    def _build_params_frame(self):
        seg = ttk.LabelFrame(self, text="パラメータ")
        seg.pack(padx=10, pady=6, fill="x")
        self.var_step_mm   = tk.StringVar(value=str(DEFAULT_STEP_MM))
        self.var_lookahead = tk.StringVar(value=str(DEFAULT_LOOKAHEAD))
        self.var_v_tcp     = tk.StringVar(value=str(DEFAULT_V_TCP))
        self.var_v_ori     = tk.StringVar(value=str(DEFAULT_V_ORI))
        self.var_p_tcp     = tk.StringVar(value=str(DEFAULT_P_TCP))
        self.var_p_ori     = tk.StringVar(value=str(DEFAULT_P_ORI))
        self.var_z_ori     = tk.StringVar(value=str(DEFAULT_Z_ORI))
        self.var_final_p   = tk.StringVar(value=str(DEFAULT_FINAL_P_TCP))
        self.var_final_fine= tk.BooleanVar(value=DEFAULT_FINAL_FINE)

        self.var_thresh_any = tk.StringVar(value=str(DEFAULT_THRESH_ANY_MM))
        self.var_thresh_all = tk.StringVar(value=str(DEFAULT_THRESH_ALL_MM))
        self.var_pause_sec  = tk.StringVar(value=str(DEFAULT_PAUSE_AFTER_EVENT_SEC))

        grid = [
            ("step(mm):", self.var_step_mm),
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
        # 閾値と停止時間
        ttk.Label(seg, text="閾値② any<= (mm):").grid(row=3, column=0, sticky="e", padx=4)
        ttk.Entry(seg, textvariable=self.var_thresh_any, width=8).grid(row=3, column=1, sticky="w")
        ttk.Label(seg, text="閾値③ all<= (mm):").grid(row=3, column=2, sticky="e", padx=4)
        ttk.Entry(seg, textvariable=self.var_thresh_all, width=8).grid(row=3, column=3, sticky="w")
        ttk.Label(seg, text="①②③時の停止時間(s):").grid(row=4, column=0, sticky="e", padx=4)
        ttk.Entry(seg, textvariable=self.var_pause_sec, width=8).grid(row=4, column=1, sticky="w")

    def _build_arduino_frame(self):
        fra = ttk.LabelFrame(self, text="Arduino信号（シミュレーション）")
        fra.pack(padx=10, pady=6, fill="x")
        self._pin_vars = [tk.StringVar(value="-1") for _ in range(9)]
        for i in range(9):
            ttk.Label(fra, text=f"pin{i+1}:").grid(row=i//3, column=(i%3)*3, padx=4, pady=2, sticky="e")
            ttk.Entry(fra, textvariable=self._pin_vars[i], width=6).grid(row=i//3, column=(i%3)*3+1, padx=2, pady=2, sticky="w")
        # 送信：保持モードに入れる（保存完了まで出し続ける）
        ttk.Button(fra, text="↑の9値を送信（保存完了まで保持）", command=self._send_hold_until_saved).grid(row=3, column=0, columnspan=6, padx=6, pady=6, sticky="w")
        # 参考：5回だけ送る旧仕様も残置（必要なら使用）
        ttk.Button(fra, text="（参考）5回だけ送信", command=self._send_override_5times).grid(row=3, column=3, columnspan=3, padx=6, pady=6, sticky="e")
        self.var_last_pins = tk.StringVar(value="最新受信: [-1,-1,-1,-1,-1,-1,-1,-1,-1]")
        ttk.Label(fra, textvariable=self.var_last_pins, foreground="#444").grid(row=4, column=0, columnspan=6, sticky="w", padx=6, pady=2)
        self.after(100, self._update_last_pins_label)

    def _build_buttons(self):
        btns = ttk.Frame(self); btns.pack(padx=10, pady=10, fill="x")
        ttk.Button(btns, text="開始", command=self.on_start).pack(side="left", padx=6)
        self.btn_pause = ttk.Button(btns, text="一時停止", command=self.on_pause_resume)
        self.btn_pause.pack(side="left", padx=6)
        ttk.Button(btns, text="次へ（スキップ）", command=self.on_skip).pack(side="left", padx=6)
        ttk.Button(btns, text="計画を再読込", command=self.on_reload_plan).pack(side="left", padx=6)

    # === ログUIハンドラ ===
    def _on_log_mode_change(self):
        mode = self.var_log_mode.get()
        self.btn_choose_log.config(state=("normal" if mode == "append" else "disabled"))

    def _choose_log_file(self):
        path = filedialog.askopenfilename(
            title="追記先のログファイルを選択",
            filetypes=[("JSON files", "*.json"), ("All files", "*.*")]
        )
        if path:
            self.var_log_path.set(path)

    # === Arduino GUI ===
    def _send_hold_until_saved(self):
        try:
            vals = [int(v.get()) for v in self._pin_vars]
        except ValueError:
            messagebox.showerror("入力エラー", "9つのピン値は整数で入力してください（-1可）")
            return
        if len(vals) != 9:
            messagebox.showerror("入力エラー", "9つすべての値を入力してください")
            return
        self.arduino.hold_until_release(vals)

    def _send_override_5times(self):
        try:
            vals = [int(v.get()) for v in self._pin_vars]
        except ValueError:
            messagebox.showerror("入力エラー", "9つのピン値は整数で入力してください（-1可）")
            return
        if len(vals) != 9:
            messagebox.showerror("入力エラー", "9つすべての値を入力してください")
            return
        self.arduino.set_override_5times(vals)

    def _update_last_pins_label(self):
        pins, ts = self.arduino.get_latest()
        self.var_last_pins.set(f"最新受信: {pins}")
        self.after(120, self._update_last_pins_label)

    # === 接続/初期化 ===
    def _auto_connect_and_init(self):
        try:
            print("[INFO] Connecting...")
            self.client.connect(timeout=5.0)
            self.client.ping()
            self._load_plan_from_file()
            messagebox.showinfo("接続", "ロボットに接続し，計画を読み込みました。")
        except Exception as e:
            messagebox.showerror("接続エラー", f"{e}")

    def _load_plan_from_file(self):
        if not os.path.exists(PUSH_PLAN_PATH):
            raise FileNotFoundError(f"push_plan.json が見つかりません: {PUSH_PLAN_PATH}")
        self.sequence = load_plan_sequence(PUSH_PLAN_PATH)
        self.seq_idx = 0
        self._update_current_target_label()

    # === 操作ボタン ===
    def on_start(self):
        if self.motion_thread and self.motion_thread.is_alive():
            messagebox.showwarning("実行中", "既に移動中です。一時停止やスキップを使用してください。")
            return

        # ログモード決定
        mode = self.var_log_mode.get()
        if mode == "none":
            self.log_file_path = None
        elif mode == "new":
            self.log_file_path = make_log_filename(LOG_DIR)
            write_log_entry(self.log_file_path, {"type":"session_start","timestamp": timestamp_str()}, mode="new")
        elif mode == "append":
            sel = self.var_log_path.get().strip()
            if not sel:
                messagebox.showerror("ログ設定", "追記先のログファイルを参照で選択してください。")
                return
            self.log_file_path = sel
            write_log_entry(self.log_file_path, {"type":"session_resume","timestamp": timestamp_str()}, mode="append")
        else:
            messagebox.showerror("ログ設定", f"未知のログモード: {mode}")
            return

        # 速度/ゾーン適用
        try:
            v_tcp = float(self.var_v_tcp.get())
            v_ori = float(self.var_v_ori.get())
            p_tcp = float(self.var_p_tcp.get())
            p_ori = float(self.var_p_ori.get())
            z_ori = float(self.var_z_ori.get())
            self.client.set_speed(v_tcp, v_ori)
            self.client.set_zone(p_tcp, p_ori, z_ori)
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

    # === ヘルパ ===
    def _update_current_target_label(self):
        if 0 <= self.seq_idx < len(self.sequence):
            label, (x,y,z,r,p,yaw), _id = self.sequence[self.seq_idx]
            self.var_curr_label.set(label)
            self.var_curr_xyzrpy.set(
                f"x={x:.1f}, y={y:.1f}, z={z:.1f}, roll={r:.1f}, pitch={p:.1f}, yaw={yaw:.1f}"
            )
        else:
            self.var_curr_label.set("-")
            self.var_curr_xyzrpy.set("x= -, y= -, z= -, roll= -, pitch= -, yaw= -")

    def _log_event_xyzrpy(self, event_type: str, target_label: str, target_pose_xyzrpy, actual_xyzrpy, pins_snapshot):
        if not self.log_file_path:
            return
        entry = {
            "type": event_type,
            "timestamp": timestamp_str(),
            "target": {"id": target_label, "xyzrpy": [float(v) for v in target_pose_xyzrpy]},
            "actual": {"xyzrpy": [float(v) for v in actual_xyzrpy]},
            "pins": [int(v) for v in pins_snapshot]
        }
        write_log_entry(self.log_file_path, entry, mode="append")

    # === メイン実行ループ ===
    def _run_plan_loop(self):
        try:
            while self.seq_idx < len(self.sequence):
                while self.pause_flag.is_set():
                    time.sleep(0.03)

                label, pose, this_id = self.sequence[self.seq_idx]
                self._update_current_target_label()

                ok = self._run_single_target_with_arduino(label, pose, this_id)
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

    # === 目標実行（①②③＋同期保存＋自動再開） ===
    def _run_single_target_with_arduino(self, label: str, target_xyzrpy, target_id: int):
        """
        - A/B問わず通常のパイプラインMoveLを実行
        - 実行中，Arduinoピンに対し条件②③を監視
          → 成立：押し込み停止→待機→pins+XYZRPYの同期スナップショット→保存→A退避→Bスキップ→続行
        - B到達(終点ACK完了)で条件①：押し込み停止→待機→同期スナップショット→保存→続行
        - RecoverableCommError は自動で再試行（手動再開不要）
        """
        while True:  # 自動リトライループ
            try:
                step_mm   = float(self.var_step_mm.get());   assert step_mm > 0
                lookahead = int(float(self.var_lookahead.get())); assert lookahead >= 1
                final_fine = bool(self.var_final_fine.get())
                final_p_tcp= float(self.var_final_p.get())
                pause_sec  = max(0.0, float(self.var_pause_sec.get()))
                if final_fine and final_p_tcp <= 0:
                    raise ValueError("終点P_TCPは正の値にしてください。")

                thresh_any = float(self.var_thresh_any.get())
                thresh_all = float(self.var_thresh_all.get())

                # 現在姿勢
                self.client.flush_recv()
                cx, cy, cz, *_ = self.client.get_cart()

                tx, ty, tz, rr, pp, yw = target_xyzrpy
                via_xyz = make_via_points_by_step((cx,cy,cz), (tx,ty,tz), step_mm=step_mm)
                n = len(via_xyz)

                self.client.start_ack_drain()

                if n == 1:
                    # 終点のみ
                    self.client.stop_ack_drain()
                    self.client.flush_recv()
                    self._move_final_with_optional_fine((tx,ty,tz), (rr,pp,yw), final_fine, final_p_tcp)

                    # ① B到達 → 停止→待機→同期保存
                    if label.startswith("B"):
                        self._pause_wait_and_sync_log(event_type="arrived_B",
                                                      target_label=label,
                                                      target_pose=target_xyzrpy,
                                                      pause_sec=pause_sec)
                    return True

                going_to_B = label.startswith("B")
                # このIDのA姿勢を退避先として見つける
                retreat_A_pose = None
                for L,(ax,ay,az,ar,ap,ayw),id_ in self.sequence:
                    if id_ == target_id and L.startswith("A"):
                        retreat_A_pose = (ax,ay,az,ar,ap,ayw)
                        break

                # 先行投入
                preload = min(lookahead, max(0, n-1))
                idx_sent = -1
                for k in range(preload):
                    if self._check_pause_skip():
                        self.client.stop_ack_drain(); self.client.flush_recv()
                        return None if self.skip_flag.is_set() else False
                    vx, vy, vz = via_xyz[k]
                    self.client.moveL_send_only(vx, vy, vz, (rr,pp,yw))
                    idx_sent = k
                    # ②③チェック
                    if going_to_B:
                        trig, pins_snapshot = self._check_pin_triggers_with_snapshot(thresh_any, thresh_all)
                        if trig:
                            self._handle_pin_trigger_during_B(label, target_xyzrpy, retreat_A_pose,
                                                              pins_snapshot, pause_sec)
                            return True  # Bスキップ扱い

                # パイプ回し
                while idx_sent < n-2:
                    if self._check_pause_skip():
                        self.client.stop_ack_drain(); self.client.flush_recv()
                        return None if self.skip_flag.is_set() else False
                    idx_sent += 1
                    nxt = idx_sent
                    if nxt < n-1:
                        vx, vy, vz = via_xyz[nxt]
                        self.client.moveL_send_only(vx, vy, vz, (rr,pp,yw))
                    time.sleep(0.04)
                    # ②③チェック
                    if going_to_B:
                        trig, pins_snapshot = self._check_pin_triggers_with_snapshot(thresh_any, thresh_all)
                        if trig:
                            self._handle_pin_trigger_during_B(label, target_xyzrpy, retreat_A_pose,
                                                              pins_snapshot, pause_sec)
                            return True

                # 終点ACK
                self.client.stop_ack_drain()
                self.client.flush_recv()
                if self._check_pause_skip():
                    return None if self.skip_flag.is_set() else False

                self._move_final_with_optional_fine((tx,ty,tz), (rr,pp,yw), final_fine, final_p_tcp)

                # ① B到達 → 停止→待機→同期保存
                if label.startswith("B"):
                    self._pause_wait_and_sync_log(event_type="arrived_B",
                                                  target_label=label,
                                                  target_pose=target_xyzrpy,
                                                  pause_sec=pause_sec)

                return True

            except RecoverableCommError as e:
                # 自動再開：ドレイン停止→フラッシュ→小休止→ループ頭から再試行
                try:
                    self.client.stop_ack_drain()
                    self.client.flush_recv()
                except:
                    pass
                time.sleep(0.2)
                continue  # user操作不要で自動再開

            except Exception:
                try:
                    self.client.stop_ack_drain()
                    self.client.flush_recv()
                except:
                    pass
                raise

    # === ②③の判定（スナップショット付き） ===
    def _check_pin_triggers_with_snapshot(self, thresh_any, thresh_all):
        pins,_ = self.arduino.get_latest()
        vals = [v if v >= 0 else 9999 for v in pins]
        any_cond = any(v <= thresh_any for v in vals)
        all_cond = all(v <= thresh_all for v in vals)
        return (any_cond or all_cond), pins  # pins はスナップショット

    # === ②③: Bへ向かう途中で発生（停止→待機→同期保存→退避→スキップ） ===
    def _handle_pin_trigger_during_B(self, label_B, target_pose_B, retreat_A_pose, pins_snapshot, pause_sec):
        # パイプ停止
        try:
            self.client.stop_ack_drain()
            self.client.flush_recv()
        except:
            pass

        # 待機（押し込み中断を可視化）
        time.sleep(pause_sec)

        # 同期スナップショット（pinsはすでにsnapshot済み）
        ax,ay,az,qx,qy,qz,qw = self.client.get_cart()
        r_act,p_act,y_act = quat_to_rpy_deg(qx,qy,qz,qw)
        actual_xyzrpy = (ax,ay,az,r_act,p_act,y_act)
        self._log_event_xyzrpy("pin_trigger", label_B, target_pose_B, actual_xyzrpy, pins_snapshot)

        # 保存完了→保持解除
        self.arduino.release_hold()

        # 退避（Aに戻る）
        if retreat_A_pose is not None:
            ax2,ay2,az2,ar2,ap2,ayw2 = retreat_A_pose
            self.client.moveL_ack(ax2, ay2, az2, (ar2,ap2,ayw2))

    # === ①: B到達時（停止→待機→同期保存） ===
    def _pause_wait_and_sync_log(self, event_type, target_label, target_pose, pause_sec):
        # パイプは既に閉じている想定だが，念のため
        try:
            self.client.stop_ack_drain()
            self.client.flush_recv()
        except:
            pass

        time.sleep(pause_sec)

        # pins は保存時点で取得（保持中なら変わらない）
        pins_snapshot,_ = self.arduino.get_latest()
        ax,ay,az,qx,qy,qz,qw = self.client.get_cart()
        r_act,p_act,y_act = quat_to_rpy_deg(qx,qy,qz,qw)
        actual_xyzrpy = (ax,ay,az,r_act,p_act,y_act)

        self._log_event_xyzrpy(event_type, target_label, target_pose, actual_xyzrpy, pins_snapshot)

        # 保存完了→保持解除（到達ログでも解除しておく）
        self.arduino.release_hold()

    # === pause/skip確認 ===
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
                self.client.moveL_ack(final_xyz[0], final_xyz[1], final_xyz[2], target_rpy)
            finally:
                time.sleep(0.02)
                self.client.set_zone(p_tcp, p_ori, z_ori)
        else:
            self.client.moveL_ack(final_xyz[0], final_xyz[1], final_xyz[2], target_rpy)

    # === 終了処理 ===
    def destroy(self):
        try:
            self.arduino.stop()
        except:
            pass
        try:
            self.client.close()
        except:
            pass
        super().destroy()


if __name__ == "__main__":
    app = App()
    app.mainloop()
