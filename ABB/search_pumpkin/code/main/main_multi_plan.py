# -*- coding: utf-8 -*-
# main/main.py  (Arduino実機シリアル監視・常時GUI反映・自動接続・通信自動再開・厳密接続判定)
# 概要:
#  - info/push_plan.json を A1,B1,A1,A2,B2,A2,... の順に実行
#  - Arduino(実機)の9ピン値をCOMポートから常時監視（開始前からGUI反映）
#  - -1以外の値はピン毎に赤色表示，未接続時は灰色ダッシュ表示＋未接続メッセージ
#  - ①B到達 / ②任意ピン<=閾値1 / ③全ピン<=閾値2 のいずれかで:
#       押し込み中断→設定秒待機→その時点の pins(9) と ロボットXYZRPY を同期取得→log保存
#       ②③はAに退避して当該Bをスキップ
#  - RecoverableCommError は自動再試行（手動再開不要）
#  - GUI: ログ保存モード（新規/追記/保存しない），COM選択/接続，ピン値モニタ，COM自動接続

import os, sys, json, math, time, threading
import tkinter as tk
from tkinter import ttk, messagebox, filedialog

# ==== パス設定 ====
BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))  # project/
if BASE_DIR not in sys.path:
    sys.path.append(BASE_DIR)

from include.robot_client import RobotClient, RecoverableCommError
from include.utils_json import make_log_filename, write_log_entry, timestamp_str

# ==== 依存（pyserial） ====
try:
    import serial
    import serial.tools.list_ports as list_ports
except Exception:
    serial = None
    list_ports = None

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
DEFAULT_BAUD = 115200

# ==== ピン閾値（GUIで変更可能）====
DEFAULT_THRESH_ANY_MM = 5.0   # ② いずれかのピンが <= この値
DEFAULT_THRESH_ALL_MM = 10.0  # ③ すべてのピンが <= この値

# ==== ①②③時の停止時間（GUIで変更可能）====
DEFAULT_PAUSE_AFTER_EVENT_SEC = 1.0

# ==== ファイル ====
PUSH_PLAN_PATH = os.path.join(BASE_DIR, "info", "push_plan.json")
LOG_DIR        = os.path.join(BASE_DIR, "log")

# --- GUIメッセージをターミナルにも出すラッパ ---
def gui_info(title: str, msg: str):
    print(f"[INFO][{title}] {msg}")
    try:
        messagebox.showinfo(title, msg)
    except Exception:
        pass

def gui_warn(title: str, msg: str):
    print(f"[WARN][{title}] {msg}")
    try:
        messagebox.showwarning(title, msg)
    except Exception:
        pass

def gui_error(title: str, msg: str):
    print(f"[ERROR][{title}] {msg}")
    try:
        messagebox.showerror(title, msg)
    except Exception:
        pass
# --- 追加：パース＋クランプ＋デバッグ ---
# ==== sanitize shim (drop-in) ====
import inspect

def _parse_float_guard5(s, default, lo, hi, name="value"):
    """
    GUI文字列 s を float に変換。失敗時は default。
    lo..hi にクランプして返す。デバッグ出力付き。
    """
    try:
        txt = str(s).strip().replace(",", "")
        txt = txt.translate(str.maketrans("．，－＋", ".-,+"))
        v = float(txt)
    except Exception:
        v = float(default)
        print(f"[DEBUG] parse fail for {name}='{s}', use default={v}")
    if v < lo:
        print(f"[DEBUG] clamp {name}: {v} -> {lo}")
        v = lo
    if v > hi:
        print(f"[DEBUG] clamp {name}: {v} -> {hi}")
        v = hi
    return v

# 互換ラッパ：4引数/5引数どちらの呼び出しでも受け付ける
def _parse_float_guard(*args, **kwargs):
    if len(args) == 4 and not kwargs:
        s, default, lo, hi = args
        return _parse_float_guard5(s, default, lo, hi, name="compat")
    return _parse_float_guard5(*args, **kwargs)

def _sanitize_zone_values(p_tcp_str, p_ori_str, z_ori_str):
    """
    Zone 入力を実用範囲に丸める（安全域：P_TCP 0.1..50、角度 0.1..100）
    """
    p_tcp = _parse_float_guard(p_tcp_str, 1.5, 0.1, 50.0,  "P_TCP")
    p_ori = _parse_float_guard(p_ori_str, 8.0, 0.1, 100.0, "P_ORI")
    z_ori = _parse_float_guard(z_ori_str, 8.0, 0.1, 100.0, "Z_ORI")
    print(f"[DEBUG] ZONE sanitized -> p_tcp={p_tcp}, p_ori={p_ori}, z_ori={z_ori}")
    return p_tcp, p_ori, z_ori

print(f"[DEBUG] installed _parse_float_guard shim; signature={inspect.signature(_parse_float_guard)}")
# ==== end sanitize shim ====



# === 差分送信用（新規追加） ==============================================
# === 差分送信用（置き換え版） ==============================================
class ZoneSpeedCache:
    """
    set_speed/set_zone を '値が変わる時だけ' 実行。
    さらに set_zone は MoveL 直後の遅延ACK(1 1 ...)に強いリトライ付き。
    """
    def __init__(self, client):
        self.c = client
        self._last_speed = None   # (v_tcp, v_ori)
        self._last_zone  = None   # (p_tcp, p_ori, z_ori)

    def set_speed_if_changed(self, v_tcp, v_ori):
        cur = (float(v_tcp), float(v_ori))
        if self._last_speed != cur:
            print(f"[DEBUG] ZoneSpeedCache.set_speed({cur})")
            self.c.set_speed(v_tcp, v_ori)
            self._last_speed = cur

    def set_zone_if_changed(self, p_tcp, p_ori, z_ori, *, force: bool=False, context: str=""):
        """
        force=True ならキャッシュ無視で必ず送る。
        送信前に小待ち→flush、失敗時は ping→flush→再送。
        """
        cur = (float(p_tcp), float(p_ori), float(z_ori))
        if (not force) and (self._last_zone == cur):
            return

        def _try_once(tag):
            # 直前MoveLの遅延ACKを吐かせる小待ち＋flush
            time.sleep(0.03)
            self.c.flush_recv()
            print(f"[DEBUG] ZoneSpeedCache.set_zone{tag} -> {cur} {('['+context+']') if context else ''}")
            self.c.set_zone(p_tcp, p_ori, z_ori)

        try:
            _try_once("(1st)")
        except RecoverableCommError as e:
            # 典型: last_raw が "1 1 0 0" で 9 が来ずにタイムアウト
            print(f"[WARN] set_zone first attempt failed: {e}. retry with ping+flush")
            try:
                self.c.ping()
            except Exception:
                pass
            time.sleep(0.03)
            self.c.flush_recv()
            _try_once("(2nd)")

        self._last_zone = cur
# =======================================================================


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


# --- main.py にコピペ（import の下あたり） -------------------------------
from dataclasses import dataclass
from typing import List, Tuple, Optional, Callable

@dataclass
class PoseRPY:
    x: float; y: float; z: float
    roll: float; pitch: float; yaw: float

def _almost_equal_pose(p1: PoseRPY, p2: PoseRPY, pos_eps=0.05, ang_eps_deg=0.3) -> bool:
    return (abs(p1.x - p2.x) <= pos_eps and
            abs(p1.y - p2.y) <= pos_eps and
            abs(p1.z - p2.z) <= pos_eps and
            abs(p1.roll - p2.roll) <= ang_eps_deg and
            abs(p1.pitch - p2.pitch) <= ang_eps_deg and
            abs(p1.yaw - p2.yaw) <= ang_eps_deg)

class PipelineMover:
    """
    ・Speed/Zoneは差分送信
    ・MoveLはウィンドウ方式（先行 send_only、末尾のみ ACK）
    ・最終点直前だけ Zone を停止寄りに切替
    ・途中中断したい時は check_abort() を渡す（Trueで即中断）
    """
    def __init__(self, client,
                 zs_cache: ZoneSpeedCache,
                 v_tcp: float = 40.0, v_ori: float = 80.0,
                 zone_blend: Tuple[float,float,float] = (1.5, 8.0, 8.0),
                 zone_stop:  Tuple[float,float,float] = (0.5, 8.0, 8.0),
                 window: int = 3):
        self.c = client
        self.zs = zs_cache
        self.v_tcp = float(v_tcp)
        self.v_ori = float(v_ori)
        self.zone_blend = tuple(float(x) for x in zone_blend)
        self.zone_stop  = tuple(float(x) for x in zone_stop)
        self.window = max(1, int(window))

    def prelude(self, tool_txyz=(0,0,0), tool_rpy=(0,0,0), use_identity_wobj=True):
        self.c.ping()
        self.c.set_wobj(identity=use_identity_wobj)
        self.c.set_tool(tool_txyz, tool_rpy)
        self.zs.set_speed_if_changed(self.v_tcp, self.v_ori)
        self.zs.set_zone_if_changed(*self.zone_blend)

    def run(self, segments: List[PoseRPY], check_abort: Optional[Callable[[], bool]] = None):
        # 連続同一点の除去
        filtered: List[PoseRPY] = []
        for seg in segments:
            if not filtered or not _almost_equal_pose(seg, filtered[-1]):
                filtered.append(seg)
        segments = filtered

        N = len(segments)
        if N == 0:
            print("[INFO] No segments."); return

        # 先行投入
        prefill = max(0, min(self.window - 1, N - 1))
        for k in range(prefill):
            if check_abort and check_abort(): return
            s = segments[k]
            self.c.moveL_send_only(s.x, s.y, s.z, (s.roll, s.pitch, s.yaw), tiny_pause=0.0)

        i = prefill
        while i < N:
            if check_abort and check_abort(): return
            s = segments[i]

            # 最終点だけ停止寄り
            if i == N - 1:
                # ここを変更（force=True, context付き）
                self.zs.set_zone_if_changed(*self.zone_stop, force=True, context="final-stop")
            else:
                self.zs.set_zone_if_changed(*self.zone_blend, context="blend")

            # 末尾はACKで同期
            self.c.moveL_ack(s.x, s.y, s.z, (s.roll, s.pitch, s.yaw))

            # 次を補充
            i += 1
            if i < N:
                if check_abort and check_abort(): return
                n = segments[i]
                self.c.moveL_send_only(n.x, n.y, n.z, (n.roll, n.pitch, n.yaw), tiny_pause=0.0)
# =======================================================================

# --- main.py にコピペ（上のクラスの直後など） --------------------------
def plan_to_segments(plan_items, use_order=("A","B")) -> list[PoseRPY]:
    """
    plan_items: 例）[{ "id":1, "A":{"x":..,"y":..,"z":..,"roll":..,"pitch":..,"yaw":..},
                          "B":{...}}, ...]
    use_order: ("A","B") なら A→B→A→B… の順で抽出
    """
    segs: list[PoseRPY] = []
    for item in plan_items:
        for key in use_order:
            p = item.get(key)
            if not p: 
                continue
            segs.append(PoseRPY(
                float(p["x"]), float(p["y"]), float(p["z"]),
                float(p["roll"]), float(p["pitch"]), float(p["yaw"])
            ))
    return segs
# -------------------------------------------------------------------------


# ==== Arduinoシリアルモニタ ====
class ArduinoSerialMonitor:
    """
    Windows 11 の COM ポートから "1:23,2:-1,..." 形式を読む。
    最新9値を保持し、get_latest() で取得可。未接続でも安全に呼べる。
    """
    def __init__(self):
        self._ser = None
        self._thread = None
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._curr = [-1]*9
        self._ts = 0.0
        self._status = "serial: disconnected"

    def list_ports(self):
        if list_ports is None:
            return []
        return [p.device for p in list_ports.comports()]

    def is_connected(self) -> bool:
        """物理的にポートが開いているか厳密判定"""
        return (self._ser is not None) and getattr(self._ser, "is_open", False)

    def connect(self, port: str, baud: int = DEFAULT_BAUD):
        if serial is None:
            raise RuntimeError("pyserial が見つかりません．`pip install pyserial` を実行してください。")
        self.disconnect()
        try:
            self._ser = serial.Serial(
                port=port,
                baudrate=baud,
                timeout=1.0,          # 1秒に拡大（安定性向上）
                write_timeout=1.0,
                rtscts=False,
                dsrdtr=False
            )
            # 自動リセット対策（Due 等）
            time.sleep(1.5)
            try:
                self._ser.reset_input_buffer()
            except:
                pass

            self._stop.clear()
            self._thread = threading.Thread(target=self._loop, daemon=True)
            self._thread.start()
            self._status = f"serial: connected ({port} @ {baud})"
        except Exception as e:
            self._ser = None
            self._status = f"serial: error ({e})"
            raise

    def disconnect(self):
        self._stop.set()
        if self._thread and self._thread.is_alive():
            try:
                self._thread.join(timeout=0.5)
            except:
                pass
        self._thread = None
        if self._ser:
            try:
                self._ser.close()
            except:
                pass
        self._ser = None
        self._status = "serial: disconnected"

    def status(self) -> str:
        return self._status

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


# ==== GUI/アプリ本体 ====
class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("open_abb plan runner + Arduino Serial Monitor")
        self.geometry("980x780")

        # ---- Robot client ----
        self.client = RobotClient(ROBOT_IP, ROBOT_PORT)
        self.zs = ZoneSpeedCache(self.client)   # ★追加：差分送信キャッシュ


        # ---- 状態 ----
        self.sequence = []
        self.seq_idx = 0
        self.pause_flag = threading.Event()
        self.skip_flag  = threading.Event()
        self.motion_thread = None
        self.log_file_path = None

        # ---- Arduino Serial ----
        self.arduino = ArduinoSerialMonitor()

        # ---- 上段: ログ保存モード ----
        self._build_log_mode_frame()

        # ---- 中段: 現在の目的地表示 ----
        self._build_current_target_frame()

        # ---- ★ 追加：push_plan選択GUI ----
        self._build_plan_select_frame()
        self._rescan_plans()

        # ---- パラメータ（速度・ゾーン・分割・閾値・停止時間）----
        self._build_params_frame()

        # ---- ArduinoシリアルGUI ----
        self._build_arduino_frame()

        # ---- 操作ボタン ----
        self._build_buttons()

        # ---- 自動接続＆初期化 ----
        self.after(100, self._auto_connect_and_init)
        # ---- シリアル自動メンテ（開始前から常時） ----
        self.after(1000, self._auto_serial_maint)
        # ---- ピン表示（開始前から常時） ----
        self.after(120, self._update_pins_display)

    # === Appクラス内：__init__の直前でも可（クラス内メソッドとして定義） ===
    def _find_push_plan_files(self, root_dir: str, limit: int = 200):
        """
        root_dir以下を再帰走査し、ファイル名に 'push_plan' を含み、拡張子が .json のファイルを列挙。
        優先順位： (1) info/push_plan.json, (2) info/配下, (3) それ以外
        返り値: [(表示名, 絶対パス), ...]
        """
        candidates = []
        for dirpath, _, filenames in os.walk(root_dir):
            for fn in filenames:
                if fn.lower().endswith(".json") and ("push_plan" in fn.lower()):
                    full = os.path.join(dirpath, fn)
                    candidates.append(full)
                    if len(candidates) >= limit:
                        break
            if len(candidates) >= limit:
                break

        # 優先ソート
        def _score(p):
            # info/push_plan.json を最優先、その次は info/配下
            rel = os.path.relpath(p, root_dir).replace("\\", "/")
            s = 1000
            if rel == "info/push_plan.json":
                s = 0
            elif rel.startswith("info/"):
                s = 100
            return (s, rel.lower())

        candidates.sort(key=_score)

        items = []
        for full in candidates:
            rel = os.path.relpath(full, root_dir).replace("\\", "/")
            # 表示名は相対パスにする
            items.append((rel, full))
        return items

    def _build_plan_select_frame(self):
        frm = ttk.LabelFrame(self, text="実行する push_plan の選択")
        frm.pack(padx=10, pady=6, fill="x")

        self._plan_items = []   # [(display, abspath), ...]
        self.var_plan_choice = tk.StringVar(value="")  # Combobox 表示用
        self.var_plan_loaded = tk.StringVar(value="—") # 「現在ロード中の計画」表示
        self.loaded_plan_path = None                   # 実際に sequence に反映済みのパス

        ttk.Label(frm, text="push_plan を含む *.json:").grid(row=0, column=0, sticky="e", padx=6, pady=4)
        self.cmb_plan = ttk.Combobox(frm, width=60, state="readonly", textvariable=self.var_plan_choice, values=[])
        self.cmb_plan.grid(row=0, column=1, sticky="w", padx=4, pady=4)

        ttk.Button(frm, text="再走査", command=self._rescan_plans).grid(row=0, column=2, padx=6, pady=4)

        # 現在ロード中のプラン表示
        ttk.Label(frm, text="現在ロード中:").grid(row=1, column=0, sticky="e", padx=6)
        ttk.Label(frm, textvariable=self.var_plan_loaded, foreground="#555").grid(row=1, column=1, sticky="w", padx=4)

    def _rescan_plans(self):
        items = self._find_push_plan_files(BASE_DIR)
        self._plan_items = items
        disp_list = [d for (d, _) in items]
        self.cmb_plan["values"] = disp_list

        # 既定値の選択ロジック
        # 1) info/push_plan.json があればそれ
        # 2) なければ最初の候補
        default_sel = None
        for d, p in items:
            if d == "info/push_plan.json":
                default_sel = d
                break
        if not default_sel and disp_list:
            default_sel = disp_list[0]

        if default_sel:
            self.var_plan_choice.set(default_sel)
        else:
            self.var_plan_choice.set("")


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
        fra = ttk.LabelFrame(self, text="Arduino（シリアル監視）")
        fra.pack(padx=10, pady=6, fill="x")
        # COM選択
        ttk.Label(fra, text="COMポート:").grid(row=0, column=0, sticky="e", padx=4, pady=2)
        self.cmb_ports = ttk.Combobox(fra, width=15, state="readonly", values=self._list_com_ports())
        self.cmb_ports.grid(row=0, column=1, sticky="w", padx=4)
        ttk.Button(fra, text="更新", command=self._refresh_com_ports).grid(row=0, column=2, padx=4)
        ttk.Label(fra, text="Baud:").grid(row=0, column=3, sticky="e", padx=4)
        self.var_baud = tk.StringVar(value=str(DEFAULT_BAUD))
        ttk.Entry(fra, textvariable=self.var_baud, width=8).grid(row=0, column=4, sticky="w", padx=2)

        self.btn_connect = ttk.Button(fra, text="接続", command=self._on_connect_serial)
        self.btn_connect.grid(row=0, column=5, padx=6)

        self.var_auto_connect = tk.BooleanVar(value=True)
        ttk.Checkbutton(fra, text="自動接続（未接続なら試行）", variable=self.var_auto_connect)\
           .grid(row=1, column=0, columnspan=3, sticky="w", padx=6, pady=2)

        self.var_serial_status = tk.StringVar(value="serial: disconnected")
        ttk.Label(fra, textvariable=self.var_serial_status, foreground="#555")\
           .grid(row=2, column=0, columnspan=6, sticky="w", padx=6, pady=2)

        # ピン値表示（-1は黒，-1以外は赤／未接続は灰色ダッシュ）
        self._pin_labels = []
        base = 3
        for i in range(9):
            ttk.Label(fra, text=f"pin{i+1}:").grid(row=base + i//3, column=(i%3)*2, padx=6, pady=3, sticky="e")
            lbl = ttk.Label(fra, text="—", foreground="#888")
            lbl.grid(row=base + i//3, column=(i%3)*2 + 1, padx=4, pady=3, sticky="w")
            self._pin_labels.append(lbl)

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

    # === Arduinoシリアル操作 ===
    def _list_com_ports(self):
        try:
            return self.arduino.list_ports()
        except:
            return []

    def _refresh_com_ports(self):
        ports = self._list_com_ports()
        self.cmb_ports["values"] = ports
        if ports and not self.cmb_ports.get():
            self.cmb_ports.set(ports[0])

    def _on_connect_serial(self):
        if self.arduino.is_connected():
            # 切断
            self.arduino.disconnect()
            self.var_serial_status.set("serial: disconnected")
            self.btn_connect.config(text="接続")
            return
        port = self.cmb_ports.get().strip()
        if not port:
            messagebox.showerror("シリアル", "COMポートを選択してください。")
            return
        try:
            baud = int(self.var_baud.get())
        except:
            messagebox.showerror("シリアル", "Baud は整数で指定してください。")
            return
        try:
            self.arduino.connect(port, baud)
            self.var_serial_status.set(self.arduino.status())
            self.btn_connect.config(text=("切断" if self.arduino.is_connected() else "接続"))
        except Exception as e:
            messagebox.showerror("シリアル接続エラー", f"{e}")
            self.var_serial_status.set(self.arduino.status())
            self.btn_connect.config(text="接続")

    def _update_pins_display(self):
        pins, _ = self.arduino.get_latest()
        connected = self.arduino.is_connected()
        for i, val in enumerate(pins):
            if not connected:
                self._pin_labels[i].config(text="—", foreground="#888")
            else:
                alarm = (isinstance(val, int) and val != -1)
                self._pin_labels[i].config(text=str(val), foreground=("red" if alarm else "black"))
        # ステータスとボタン文言を同期
        if self.arduino.is_connected():
            self.var_serial_status.set(self.arduino.status())
            self.btn_connect.config(text="切断")
        else:
            self.var_serial_status.set("serial: 未接続（COM選択→接続，または自動接続待ち）")
            self.btn_connect.config(text="接続")
        self.after(120, self._update_pins_display)

    def _auto_serial_maint(self):
        """1秒周期のシリアル常時メンテ：ポート更新＋自動接続＋ステータス整備"""
        try:
            # COM一覧を最新化
            ports = self._list_com_ports()
            current_vals = list(self.cmb_ports["values"]) if self.cmb_ports["values"] else []
            if ports != current_vals:
                self.cmb_ports["values"] = ports
                if ports and not self.cmb_ports.get():
                    self.cmb_ports.set(ports[0])

            # 自動接続：未接続 かつ チェックON
            if self.var_auto_connect.get() and not self.arduino.is_connected():
                target_port = self.cmb_ports.get().strip()
                if not target_port and ports:
                    # 「Arduino」っぽい記述を優先選択
                    cand = None
                    try:
                        import serial.tools.list_ports as lp
                        for p in lp.comports():
                            desc = (p.description or "").lower()
                            if "arduino" in desc or "usb serial" in desc or "com" in (p.device or "").lower():
                                cand = p.device
                                break
                    except:
                        cand = None
                    target_port = cand or ports[0]

                if target_port:
                    try:
                        baud = int(self.var_baud.get())
                    except:
                        baud = DEFAULT_BAUD
                    try:
                        self.arduino.connect(target_port, baud)
                    except Exception:
                        pass  # 次周期に再試行

            # ステータスとボタン文言を同期
            if self.arduino.is_connected():
                self.var_serial_status.set(self.arduino.status())
                self.btn_connect.config(text="切断")
            else:
                self.var_serial_status.set("serial: 未接続（COM選択→接続，または自動接続待ち）")
                self.btn_connect.config(text="接続")

        except Exception:
            pass
        finally:
            self.after(1000, self._auto_serial_maint)

    # === 接続/初期化 ===
    def _auto_connect_and_init(self):
        try:
            print("[INFO] Connecting...")
            self.client.connect(timeout=5.0)
            self.client.ping()
            # ★ 選択中のプランを読み込む
            self._load_plan_from_file()
            messagebox.showinfo("接続", "ロボットに接続し，計画を読み込みました。")
        except Exception as e:
            messagebox.showerror("接続エラー", f"{e}")
        self._refresh_com_ports()

    def _resolve_selected_plan_path(self) -> str:
        """
        Comboboxの表示名から絶対パスを引く。
        表示名とパスのペアは self._plan_items に保持。
        未選択時は既定の PUSH_PLAN_PATH を返す（存在すれば）。
        """
        sel = self.var_plan_choice.get().strip()
        for d, full in self._plan_items:
            if d == sel:
                return full
        # フォールバック
        if os.path.exists(PUSH_PLAN_PATH):
            return PUSH_PLAN_PATH
        # それも無ければ空文字
        return ""

    def _load_plan_from_file(self, path: str | None = None):
        """
        path が None の場合は GUI選択中のものを使用。
        読み込み成功時に self.loaded_plan_path を更新し、UIにも反映。
        """
        if not path:
            path = self._resolve_selected_plan_path()

        if not path or (not os.path.exists(path)):
            raise FileNotFoundError(f"push_plan が見つかりません: {path or '(未選択)'}")

        self.sequence = load_plan_sequence(path)
        self.seq_idx = 0
        self.loaded_plan_path = path
        rel = os.path.relpath(path, BASE_DIR).replace("\\", "/")
        self.var_plan_loaded.set(rel)
        self._update_current_target_label()


    # === 操作ボタン ===
    def on_start(self):
        if self.motion_thread and self.motion_thread.is_alive():
            messagebox.showwarning("実行中", "既に移動中です。一時停止やスキップを使用してください。")
            return
        
        # ★開始直前に、選択中プランを必ず反映
        try:
            sel_path = self._resolve_selected_plan_path()
            if not sel_path:
                messagebox.showerror("計画未選択", "push_plan を含む JSON が選択されていません。")
                return
            # 選択が現在ロード済みと異なるなら再ロード
            if sel_path != self.loaded_plan_path:
                self._load_plan_from_file(sel_path)
        except Exception as e:
            messagebox.showerror("計画読み込みエラー", f"{e}")
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
            v_tcp = _parse_float_guard(self.var_v_tcp.get(), DEFAULT_V_TCP, 1.0, 1000.0, "V_TCP")
            v_ori = _parse_float_guard(self.var_v_ori.get(), DEFAULT_V_ORI, 1.0, 1000.0, "V_ORI")
            p_tcp, p_ori, z_ori = _sanitize_zone_values(self.var_p_tcp.get(),
                                                        self.var_p_ori.get(),
                                                        self.var_z_ori.get())
            if hasattr(self, "zs"):  # ZoneSpeedCache がある場合
                print(f"[DEBUG] set_speed_if_changed({v_tcp}, {v_ori})")
                self.zs.set_speed_if_changed(v_tcp, v_ori)
                print(f"[DEBUG] set_zone_if_changed({p_tcp}, {p_ori}, {z_ori})")
                self.zs.set_zone_if_changed(p_tcp, p_ori, z_ori, context="startup")
            else:
                print(f"[DEBUG] set_speed({v_tcp}, {v_ori})")
                self.client.set_speed(v_tcp, v_ori)
                print(f"[DEBUG] set_zone({p_tcp}, {p_ori}, {z_ori})")
                self.client.set_zone(p_tcp, p_ori, z_ori)
        except Exception as e:
            gui_error("入力エラー", f"{e}")
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
            sel_path = self._resolve_selected_plan_path()
            if not sel_path:
                raise FileNotFoundError("push_plan を含む JSON が選択されていません。")
            self._load_plan_from_file(sel_path)
            rel = os.path.relpath(sel_path, BASE_DIR).replace("\\", "/")
            messagebox.showinfo("読込", f"{rel}\nを再読み込みしました。")
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
        ・経路分割→PipelineMoverでパイプライン実行
        ・②③のピントリガ時は中断してログ→Aに退避→Bをスキップ扱い
        ・B到達時は停止→待機→同期ログ
        """
        # GUI値
        step_mm   = float(self.var_step_mm.get());   assert step_mm > 0
        lookahead = int(float(self.var_lookahead.get())); assert lookahead >= 1
        final_fine = bool(self.var_final_fine.get())
        final_p_tcp= float(self.var_final_p.get())
        pause_sec  = max(0.0, float(self.var_pause_sec.get()))
        if final_fine and final_p_tcp <= 0:
            raise ValueError("終点P_TCPは正の値にしてください。")
        thresh_any = float(self.var_thresh_any.get())
        thresh_all = float(self.var_thresh_all.get())

        # 退避先A（同じIDのA）
        retreat_A_pose = None
        for L,(ax,ay,az,ar,ap,ayw),id_ in self.sequence:
            if id_ == target_id and L.startswith("A"):
                retreat_A_pose = (ax,ay,az,ar,ap,ayw); break

        # 現在位置（最初に1回だけ）
        cx, cy, cz, *_ = self.client.get_cart()
        tx, ty, tz, rr, pp, yw = target_xyzrpy

        # 経路分割（XYZは分割、姿勢は一定）
        via_xyz = make_via_points_by_step((cx,cy,cz), (tx,ty,tz), step_mm=step_mm)
        segments = [PoseRPY(x,y,z, rr,pp,yw) for (x,y,z) in via_xyz]

        # パイプラインムーバをGUI値で初期化
        v_tcp = _parse_float_guard(self.var_v_tcp.get(), DEFAULT_V_TCP, 1.0, 1000.0, "V_TCP")
        v_ori = _parse_float_guard(self.var_v_ori.get(), DEFAULT_V_ORI, 1.0, 1000.0, "V_ORI")
        p_tcp, p_ori, z_ori = _sanitize_zone_values(self.var_p_tcp.get(),
                                                    self.var_p_ori.get(),
                                                    self.var_z_ori.get())
        final_p_tcp_safe = _parse_float_guard(final_p_tcp, 0.5, 0.1, 50.0, "FINAL_P_TCP")

        zone_blend = (p_tcp, p_ori, z_ori)
        zone_stop  = ((final_p_tcp_safe if final_fine else p_tcp), p_ori, z_ori)

        print(f"[DEBUG] Pipeline zones -> blend={zone_blend}, stop={zone_stop}")
        print(f"[DEBUG] Pipeline speed -> v_tcp={v_tcp}, v_ori={v_ori}")



        mover = PipelineMover(
            self.client, self.zs,
            v_tcp=v_tcp, v_ori=v_ori,
            zone_blend=zone_blend, zone_stop=zone_stop,
            window=max(1, lookahead)
        )
        # 1回目だけの初期化は on_start 済みだが、安全に再適用（差分送信で無駄送信なし）
        mover.prelude(tool_txyz=(0,0,0), tool_rpy=(0,0,0), use_identity_wobj=True)

        going_to_B = label.startswith("B")

        # 中断条件：一時停止/スキップ or ②③ピントリガ（B時のみ）
        def _check_abort():
            # 一時停止
            while self.pause_flag.is_set():
                time.sleep(0.02)
            # スキップ
            if self.skip_flag.is_set():
                return True
            # ②③ピントリガ（Bに向かう時のみ監視）
            if going_to_B:
                pins,_ = self.arduino.get_latest()
                vals = [v if v >= 0 else 9999 for v in pins]
                if any(v <= thresh_any for v in vals) or all(v <= thresh_all for v in vals):
                    # 停止→待機→同期ログ
                    time.sleep(pause_sec)
                    pins_snapshot,_ = self.arduino.get_latest()
                    ax,ay,az,qx,qy,qz,qw = self.client.get_cart()
                    r_act,p_act,y_act = quat_to_rpy_deg(qx,qy,qz,qw)
                    actual_xyzrpy = (ax,ay,az,r_act,p_act,y_act)
                    self._log_event_xyzrpy("pin_trigger", label, target_xyzrpy, actual_xyzrpy, pins_snapshot)
                    # 退避（Aへ）
                    if retreat_A_pose is not None:
                        ax2,ay2,az2,ar2,ap2,ayw2 = retreat_A_pose
                        self.client.moveL_ack(ax2, ay2, az2, (ar2,ap2,ayw2))
                    # Bはスキップ扱い
                    self.skip_flag.set()
                    return True
            return False

        # 実行（途中で _check_abort() が True なら即中断）
        mover.run(segments, check_abort=_check_abort)

        # スキップ指示で中断されていれば None を返し、呼び出し側で次へ
        if self.skip_flag.is_set():
            self.skip_flag.clear()
            return None

        # ここまで来たら目標到達。Bなら停止→待機→同期ログ
        if going_to_B:
            time.sleep(pause_sec)
            pins_snapshot,_ = self.arduino.get_latest()
            ax,ay,az,qx,qy,qz,qw = self.client.get_cart()
            r_act,p_act,y_act = quat_to_rpy_deg(qx,qy,qz,qw)
            actual_xyzrpy = (ax,ay,az,r_act,p_act,y_act)
            self._log_event_xyzrpy("arrived_B", label, target_xyzrpy, actual_xyzrpy, pins_snapshot)

        return True
    # =======================================================================


    # === ②③の判定 ===
    def _check_pin_triggers(self, thresh_any, thresh_all):
        pins,_ = self.arduino.get_latest()
        vals = [v if v >= 0 else 9999 for v in pins]
        any_cond = any(v <= thresh_any for v in vals)
        all_cond = all(v <= thresh_all for v in vals)
        return (any_cond or all_cond), pins

    # === ②③: Bへ向かう途中で発生（停止→待機→同期保存→退避→スキップ） ===
    def _handle_pin_trigger_during_B(self, label_B, target_pose_B, retreat_A_pose, pause_sec):
        # パイプ停止
        try:
            self.client.stop_ack_drain()
            self.client.flush_recv()
        except:
            pass

        # 待機（押し込み中断を可視化）
        time.sleep(pause_sec)

        # 同期スナップショット
        pins_snapshot,_ = self.arduino.get_latest()
        ax,ay,az,qx,qy,qz,qw = self.client.get_cart()
        r_act,p_act,y_act = quat_to_rpy_deg(qx,qy,qz,qw)
        actual_xyzrpy = (ax,ay,az,r_act,p_act,y_act)
        self._log_event_xyzrpy("pin_trigger", label_B, target_pose_B, actual_xyzrpy, pins_snapshot)

        # 退避（Aに戻る）
        if retreat_A_pose is not None:
            ax2,ay2,az2,ar2,ap2,ayw2 = retreat_A_pose
            self.client.moveL_ack(ax2, ay2, az2, (ar2,ap2,ayw2))

    # === ①: B到達時（停止→待機→同期保存） ===
    def _pause_wait_and_sync_log(self, event_type, target_label, target_pose, pause_sec):
        try:
            self.client.stop_ack_drain()
            self.client.flush_recv()
        except:
            pass

        time.sleep(pause_sec)

        pins_snapshot,_ = self.arduino.get_latest()
        ax,ay,az,qx,qy,qz,qw = self.client.get_cart()
        r_act,p_act,y_act = quat_to_rpy_deg(qx,qy,qz,qw)
        actual_xyzrpy = (ax,ay,az,r_act,p_act,y_act)

        self._log_event_xyzrpy(event_type, target_label, target_pose, actual_xyzrpy, pins_snapshot)

    # === pause/skip確認 ===
    def _check_pause_skip(self):
        while self.pause_flag.is_set():
            time.sleep(0.02)
        return self.skip_flag.is_set()

    # --- 置換版：最終点だけ fine 停止（コピペ置換） ---
    def _move_final_with_optional_fine(self, final_xyz, target_rpy, final_fine, final_p_tcp):
        p_tcp, p_ori, z_ori = _sanitize_zone_values(self.var_p_tcp.get(),
                                            self.var_p_ori.get(),
                                            self.var_z_ori.get())
        final_p_tcp = _parse_float_guard(final_p_tcp, 0.5, 0.1, 50.0, "FINAL_P_TCP")
        print(f"[DEBUG] FINAL fine zone -> ({final_p_tcp}, {p_ori}, {z_ori})")


        if not final_fine:
            self.client.moveL_ack(final_xyz[0], final_xyz[1], final_xyz[2], target_rpy)
            return

        # 最終点直前だけ fine 寄りに
        self.zs.set_zone_if_changed(final_p_tcp, p_ori, z_ori)
        self.client.moveL_ack(final_xyz[0], final_xyz[1], final_xyz[2], target_rpy)
        # 元に戻す（差分送信で不要なら送られない）
        self.zs.set_zone_if_changed(p_tcp, p_ori, z_ori)
    # =======================================================================



    # === 終了処理 ===
    def destroy(self):
        try:
            self.arduino.disconnect()
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
