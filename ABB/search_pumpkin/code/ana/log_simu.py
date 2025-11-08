# -*- coding: utf-8 -*-
# log_viewer.py (rev9: 黄色線削除版)
# 依存: Python 3.8+ / tkinter / matplotlib

import json, math, os, re, sys, logging, tkinter as tk
from tkinter import ttk, filedialog, messagebox
from typing import Dict, Any, List, Tuple

# ===== ログ =====
logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")
log = logging.getLogger("pinart")

# ===== Matplotlib =====
import matplotlib
matplotlib.use("TkAgg")
from matplotlib import rcParams, font_manager as fm
import warnings

JP_FONT_CANDIDATES = [
    "Yu Gothic UI", "Yu Gothic", "Meiryo", "MS Gothic", "MS UI Gothic",
    "Arial Unicode MS", "DejaVu Sans"
]
def choose_font() -> str:
    for name in JP_FONT_CANDIDATES:
        try:
            path = fm.findfont(name, fallback_to_default=False)
            if path and os.path.exists(path):
                return name
        except Exception:
            pass
    return "DejaVu Sans"

rcParams["font.family"] = [choose_font()]
rcParams["axes.unicode_minus"] = False
warnings.filterwarnings("ignore", message=r".*findfont: Font family .* not found.*")

from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

# ===== 定数 =====
PADDING = 20.0
LOG_NAME_RE = re.compile(r"log_(\d{8})_(\d{6})\.json$")

# ===== グラフサイズ設定 =====
AUTO_MARGIN = 15.0  # [mm] 軸の四方に足すマージン

def _compute_auto_limits(xs, ys, zs, margin=AUTO_MARGIN):
    """描画候補の全点から軸範囲を作る（等尺ベース）"""
    if not xs or not ys or not zs:
        # フォールバック（空の場合）
        return (0, 100), (0, 100), (0, 100)

    xmin, xmax = min(xs), max(xs)
    ymin, ymax = min(ys), max(ys)
    zmin, zmax = min(zs), max(zs)

    # センタと最大スパンで等尺ボックスを作る
    cx, cy, cz = (xmin + xmax) / 2.0, (ymin + ymax) / 2.0, (zmin + zmax) / 2.0
    span = max(xmax - xmin, ymax - ymin, zmax - zmin)
    if span <= 0:
        span = 1.0  # ゼロ分散対策

    # マージンを加えた半幅
    half = span / 2.0 + margin

    xlim = (cx - half, cx + half)
    ylim = (cy - half, cy + half)
    zlim = (cz - half, cz + half)
    return xlim, ylim, zlim



# ===== 数学ユーティリティ =====
def rpy_deg_to_rotmat(roll, pitch, yaw):
    rx, ry, rz = map(math.radians, (roll, pitch, yaw))
    cx, sx = math.cos(rx), math.sin(rx)
    cy, sy = math.cos(ry), math.sin(ry)
    cz, sz = math.cos(rz), math.sin(rz)
    Rx = [[1,0,0],[0,cx,-sx],[0,sx,cx]]
    Ry = [[cy,0,sy],[0,1,0],[-sy,0,cy]]
    Rz = [[cz,-sz,0],[sz,cz,0],[0,0,1]]
    def mm(A,B): return [[sum(A[i][k]*B[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
    return mm(mm(Rz,Ry),Rx)

def rot_apply(R, v):
    return [R[0][0]*v[0]+R[0][1]*v[1]+R[0][2]*v[2],
            R[1][0]*v[0]+R[1][1]*v[1]+R[1][2]*v[2],
            R[2][0]*v[0]+R[2][1]*v[1]+R[2][2]*v[2]]

def transform_point(xyzrpy, p_local):
    x,y,z,r,p,yaw = xyzrpy
    R = rpy_deg_to_rotmat(r,p,yaw)
    pw = rot_apply(R, p_local)
    return [pw[0]+x, pw[1]+y, pw[2]+z]

# ===== I/O =====
def load_json(path: str) -> Dict[str, Any]:
    with open(path,"r",encoding="utf-8") as f:
        return json.load(f)

def autodetect_files() -> Tuple[str,str,str]:
    base_dir = os.path.dirname(os.path.abspath(__file__))
    latest_log, latest_key = "", None
    for fn in os.listdir(base_dir):
        m = LOG_NAME_RE.match(fn)
        if m:
            key = m.group(1)+m.group(2)
            if (latest_key is None) or (key > latest_key):
                latest_key = key
                latest_log = os.path.join(base_dir, fn)
    push = os.path.join(base_dir,"push_plan.json") if os.path.exists(os.path.join(base_dir,"push_plan.json")) else ""
    info = os.path.join(base_dir,"pinart_info.json") if os.path.exists(os.path.join(base_dir,"pinart_info.json")) else ""
    log.info(f"自動検出: log={os.path.basename(latest_log) or '(なし)'} / push_plan={'あり' if push else 'なし'} / pinart_info={'あり' if info else 'なし'}")
    return latest_log, push, info

def compute_axes_from_pushplan(plan: Dict[str, Any]) -> Tuple[Tuple[float,float],Tuple[float,float],Tuple[float,float]]:
    xs,ys,zs = [],[],[]
    for it in plan.get("plan", []):
        for key in ("A","B"):
            if key in it and len(it[key]) >= 3:
                xs.append(it[key][0]); ys.append(it[key][1]); zs.append(it[key][2])
        for key in ("Pa","Pb"):
            for pt in it.get(key, []):
                xs.append(pt[0]); ys.append(pt[1]); zs.append(pt[2])
    if not xs:
        return (0,100),(0,100),(0,100)
    return ((min(xs)-PADDING, max(xs)+PADDING),
            (min(ys)-PADDING, max(ys)+PADDING),
            (min(zs)-PADDING, max(zs)+PADDING))

# ===== 計算 =====
def compute_endpoints_for_entry(entry: Dict[str, Any], pin_info: Dict[str, Any]) -> Dict[str, Any]:
    """
    entry: {"target":{"xyzrpy", "id"...}, "actual":{"xyzrpy"...}, "pins":[L or None ...]}
    pin_info: {"pin_bases_wrist_xyz_mm":[[x,y,z]...], "pin_length_mm": float}
    戻り値:
      - target_tcp: 目標TCP座標(3)
      - actual_tcp: 実TCP座標(3)
      - pressed:   [(pin_idx, [x,y,z]), ...]  実機: 押し込み有り（contact=True相当）
      - unpressed: [(pin_idx, [x,y,z]), ...]  実機: 押し込み無し（contact=False相当）
      - full_tip_world: 目標TCP+full長の全ピン先端（参考。描画や比較には使わない）
    """
    bases = pin_info["pin_bases_wrist_xyz_mm"]
    full  = float(pin_info["pin_length_mm"])
    tgt = entry["target"]["xyzrpy"]
    act = entry["actual"]["xyzrpy"]
    pins = entry["pins"]

    # 目標: full長先端
    full_tip_world = [transform_point(tgt, [b[0], b[1], b[2] + full]) for b in bases]

    pressed, unpressed = [], []
    for i, b in enumerate(bases):
        L = None
        if i < len(pins):
            L = pins[i]
        if (L is not None) and (L >= 0):
            # 押し込みあり（Lは押し込まれた実長）
            pt = transform_point(act, [b[0], b[1], b[2] + float(L)])
            pressed.append((i + 1, pt))  # 1始まりのピン番号で整合
        else:
            pt = transform_point(act, [b[0], b[1], b[2] + full])
            unpressed.append((i + 1, pt))

    return {
        "target_tcp": tgt[:3],
        "actual_tcp": act[:3],
        "pressed": pressed,
        "unpressed": unpressed,
        "full_tip_world": full_tip_world,
    }

def make_complete_result(log_data: Dict[str, Any],
                         pin_info: Dict[str, Any],
                         axes_limits: Dict[str, Tuple[float,float]]) -> Dict[str, Any]:
    results, n_pintrigger = [], 0
    for e in log_data.get("logs", []):
        if all(k in e for k in ("target","actual","pins")):
            comp = compute_endpoints_for_entry(e, pin_info)
            rid = e.get("target",{}).get("id","")
            results.append({
                "id": rid, "type": e.get("type"), "timestamp": e.get("timestamp"),
                "target": e.get("target"), "actual": e.get("actual"), "pins": e.get("pins"),
                "computed": comp
            })
            if e.get("type") == "pin_trigger":
                n_pintrigger += 1
    log.info(f"ログ: 全{len(log_data.get('logs',[]))}件 / 対象{len(results)}件 / pin_trigger {n_pintrigger}件")
    return {"meta":{"version":"2.0"}, "axes_limits":axes_limits, "results":results}

# 追加（クラス外のユーティリティでもOKだが、Viewer内に入れると参照しやすい）
def _ids_equal(a, b) -> bool:
    """ID同士の比較を表記揺れに強くする．数字以外を除去して比較．
       例: '1' と 1，' id-001 ' と '1' を等価扱い"""
    sa = re.sub(r"\D+", "", str(a)).lstrip("0") or str(a).strip()
    sb = re.sub(r"\D+", "", str(b)).lstrip("0") or str(b).strip()
    return sa == sb

# ===== GUI本体 =====
class PinArtViewer(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Pin-Art Sensor Point Cloud Viewer (rev10)")
        self.geometry("1360x860")

        self.grid_rowconfigure(0, weight=1)
        self.grid_columnconfigure(1, weight=1)

        auto_log, auto_push, auto_info = autodetect_files()
        self.path_log     = tk.StringVar(value=auto_log)
        self.path_push    = tk.StringVar(value=auto_push)
        self.path_pininfo = tk.StringVar(value=auto_info)
        self.path_result  = tk.StringVar(value="")

        self.show_1 = tk.BooleanVar(value=False)
        self.show_2 = tk.BooleanVar(value=False)
        self.show_3 = tk.BooleanVar(value=True)
        self.show_4 = tk.BooleanVar(value=True)
        self.show_5 = tk.BooleanVar(value=True)

        self.current_result=None
        self._pin_info=None
        self.id_order = []
        self.id_to_points = {}
        self.id_with_no_press=set()
        self.scatter_to_id={}
        self.focused_id=None

        # === 左ペイン ===
        left = ttk.Frame(self, padding=6)
        left.grid(row=0, column=0, sticky="ns")
        self._add_file_row(left, "log.json（自動検出優先）", self.path_log)
        self._add_file_row(left, "push_plan.json", self.path_push)
        self._add_file_row(left, "pinart_info.json", self.path_pininfo)
        self._add_file_row(left, "結果ファイル（完全版）", self.path_result)

        frm_chk = ttk.LabelFrame(left, text="表示対象（①〜⑤）")
        frm_chk.grid(row=10, column=0, sticky="ew", pady=6)
        ttk.Checkbutton(frm_chk, text="① 目標TCP(青▲)",    variable=self.show_1, command=self.redraw_from_cache).grid(row=0,column=0,sticky="w",padx=4)
        ttk.Checkbutton(frm_chk, text="② 目標ピンfull(青○)", variable=self.show_2, command=self.redraw_from_cache).grid(row=0,column=1,sticky="w",padx=4)
        ttk.Checkbutton(frm_chk, text="③ 実TCP(赤▲)",      variable=self.show_3, command=self.redraw_from_cache).grid(row=1,column=0,sticky="w",padx=4)
        ttk.Checkbutton(frm_chk, text="④ 実ピン押込(赤○)",  variable=self.show_4, command=self.redraw_from_cache).grid(row=1,column=1,sticky="w",padx=4)
        ttk.Checkbutton(frm_chk, text="⑤ 実ピン未押(赤×)",  variable=self.show_5, command=self.redraw_from_cache).grid(row=1,column=2,sticky="w",padx=4)

        self.autoscale = tk.BooleanVar(value=True)
        self.fixed_xlim = self.fixed_ylim = self.fixed_zlim = None
        frm_zoom = ttk.LabelFrame(left, text="表示範囲")
        frm_zoom.grid(row=12, column=0, sticky="ew", pady=6)
        ttk.Checkbutton(frm_zoom, text="自動ズーム（描画要素に合わせる）",
                        variable=self.autoscale, command=self.on_toggle_autoscale).grid(row=0, column=0, sticky="w", padx=4)

        frm_btn = ttk.Frame(left)
        frm_btn.grid(row=11, column=0, sticky="ew", pady=6)
        ttk.Button(frm_btn, text="読込＆描画", command=self.load_and_plot).grid(row=0, column=0, padx=3)
        ttk.Button(frm_btn, text="結果ファイルとして保存", command=self.save_result).grid(row=0, column=1, padx=3)
        ttk.Button(frm_btn, text="クリア", command=self._clear_plot).grid(row=0, column=2, padx=3)

        # --- シミュレーション拡張（ここから）
        self.sim_overlay = SimOverlay(self)
        self._init_sim_widgets(left)
        # --- シミュレーション拡張（ここまで）

        # === 右ペイン ===
        right = ttk.Frame(self, padding=6)
        right.grid(row=0, column=2, sticky="ns")
        right.grid_rowconfigure(6, weight=1)  # ← 不一致Text領域を伸縮可能に

        frm_top = ttk.Frame(right); frm_top.grid(row=0, column=0, sticky="ew")
        ttk.Label(frm_top, text="目標値ID（クリックでフォーカス）").pack(side=tk.LEFT)
        ttk.Button(frm_top, text="クリア", command=self.reset_focus).pack(side=tk.RIGHT, padx=4)

        # ← 高さを“現状の半分”に
        self.listbox = tk.Listbox(right, height=11)
        self.listbox.grid(row=1, column=0, sticky="ns")
        self.listbox.bind("<<ListboxSelect>>", self.on_list_select)

        self.counter_var = tk.StringVar(value="押し込み回数: 0 ／ 点群総数(赤○): 0 ／ 描画点数: 0")
        ttk.Label(right, textvariable=self.counter_var).grid(row=2, column=0, sticky="w", pady=4)

        # 統計ボタン
        ttk.Button(right, text="統計を計算", command=self.on_click_compute_stats)\
           .grid(row=3, column=0, sticky="ew", pady=(2,6))

        # 統計行（1行要約表示）
        self.stats_var = tk.StringVar(value="統計: なし")
        ttk.Label(right, textvariable=self.stats_var, wraplength=280, justify="left")\
           .grid(row=4, column=0, sticky="w", pady=2)

        # 接触不一致の詳細（スクロール可能）
        ttk.Label(right, text="（接触不一致の詳細）", foreground="#555").grid(row=5, column=0, sticky="w")
        frm_mis = ttk.Frame(right); frm_mis.grid(row=6, column=0, sticky="nsew")
        frm_mis.grid_rowconfigure(0, weight=1); frm_mis.grid_columnconfigure(0, weight=1)
        self.mismatch_text = tk.Text(frm_mis, height=8, width=36, wrap="none", state="disabled")
        self.mismatch_text.grid(row=0, column=0, sticky="nsew")
        sb = ttk.Scrollbar(frm_mis, orient="vertical", command=self.mismatch_text.yview)
        sb.grid(row=0, column=1, sticky="ns")
        self.mismatch_text.configure(yscrollcommand=sb.set)

        # === 中央ペイン ===
        center = ttk.Frame(self, padding=4)
        center.grid(row=0, column=1, sticky="nsew")
        center.grid_rowconfigure(0, weight=1)
        center.grid_columnconfigure(0, weight=1)

        self.fig = Figure(figsize=(8.0,6.4), dpi=100)
        self.ax  = self.fig.add_subplot(111, projection='3d')
        self.ax.set_facecolor((1,1,1)); self.ax.grid(True)

        self.canvas = FigureCanvasTkAgg(self.fig, master=center)
        self.canvas_widget = self.canvas.get_tk_widget()
        self.canvas_widget.grid(row=0, column=0, sticky="nsew")
        self.canvas_widget.configure(width=900, height=640, borderwidth=1, relief="sunken")
        self.canvas.mpl_connect("pick_event", self.on_pick)

        self.status = tk.StringVar(value="準備完了．")
        ttk.Label(self, textvariable=self.status, anchor="w").grid(row=1, column=0, columnspan=3, sticky="ew")

        self._redraw_empty()
        if self.path_log.get() and self.path_push.get() and self.path_pininfo.get():
            self.after(120, self.load_and_plot)


    # --- UI補助 ---
    def _add_file_row(self, parent, label_text, var):
        row = ttk.Frame(parent)
        row.grid(sticky="ew", pady=2)
        row.grid_columnconfigure(1, weight=1)
        ttk.Label(row, text=label_text, width=24).grid(row=0, column=0, sticky="w")
        ttk.Entry(row, textvariable=var).grid(row=0, column=1, sticky="ew", padx=4)
        ttk.Button(row, text="参照", command=lambda: self._browse(var, label_text)).grid(row=0, column=2, padx=2)

    def _browse(self, var, title):
        path = filedialog.askopenfilename(title=title, filetypes=[("JSON","*.json"),("All","*.*")])
        if path: var.set(path)

    # --- 描画 ---
    def _force_draw(self):
        self.fig.tight_layout(); self.canvas.draw(); self.canvas_widget.update_idletasks()

    def _redraw_empty(self):
        self.ax.clear(); self.ax.set_title("Pin-Art Viewer")
        self.ax.set_xlabel("X [mm]"); self.ax.set_ylabel("Y [mm]"); self.ax.set_zlabel("Z [mm]")
        self.ax.grid(True); self._force_draw()

    def _clear_plot(self):
        self.current_result=None; self.id_order.clear(); self.id_to_points.clear()
        self.id_with_no_press.clear(); self.scatter_to_id.clear()
        self.counter_var.set("押し込み回数: 0 ／ 点群総数(赤○): 0 ／ 描画点数: 0")
        self._redraw_empty(); self.status.set("プロットをクリアした．")

    def reset_focus(self):
        """フォーカス解除"""
        self.focused_id = None
        if self.current_result:
            self._plot_from_result(self.current_result)
            self.status.set("フォーカスを解除しました．")
        self.listbox.selection_clear(0, tk.END)

    def load_and_plot(self):
        try:
            self._pin_info = None  # 比較で使う
            if self.path_result.get():
                result = load_json(self.path_result.get())
                log.info("結果ファイルから読込")
            else:
                if not (self.path_log.get() and self.path_push.get() and self.path_pininfo.get()):
                    messagebox.showwarning("警告","log / push_plan / pinart_info を指定してください．")
                    return
                log_data = load_json(self.path_log.get())
                self._pin_info = load_json(self.path_pininfo.get())   # ←保持
                push_plan = load_json(self.path_push.get())
                xlim, ylim, zlim = compute_axes_from_pushplan(push_plan)
                result = make_complete_result(log_data, self._pin_info, {"xlim":xlim,"ylim":ylim,"zlim":zlim})

            self.current_result = result
            self._plot_from_result(result)
            self.status.set("読み込み・描画完了．")
        except Exception as e:
            messagebox.showerror("エラー", f"読込/描画中にエラー：\n{e}")
            log.exception("読込＆描画で例外")


    def on_toggle_autoscale(self):
        """自動ズームのON/OFFが切り替わった時の挙動を制御する。"""
        if not self.autoscale.get():
            # OFFに切り替わる瞬間：現在の描画範囲を固定として保存
            try:
                self.fixed_xlim = tuple(self.ax.get_xlim())
                self.fixed_ylim = tuple(self.ax.get_ylim())
                self.fixed_zlim = tuple(self.ax.get_zlim())
            except Exception:
                # まだ図が無い/初回などのフォールバック
                self.fixed_xlim = self.fixed_xlim or (0, 100)
                self.fixed_ylim = self.fixed_ylim or (0, 100)
                self.fixed_zlim = self.fixed_zlim or (0, 100)
        else:
            # ONに戻った瞬間：固定範囲はいったん捨てる（その時点の内容で再自動調整）
            self.fixed_xlim = None
            self.fixed_ylim = None
            self.fixed_zlim = None

        # いまの状態で再描画（ONなら自動調整して描画、OFFなら固定値で描画）
        self.redraw_from_cache()

    def redraw_from_cache(self):
        if self.current_result: self._plot_from_result(self.current_result, focused_id=self.focused_id)

    @staticmethod
    def _id_numeric_key(rid: str):
        if isinstance(rid,str) and rid and rid[0].upper()=="B":
            try: return (int(rid[1:]), rid)
            except ValueError: pass
        return (10**9, rid or "")

    def _plot_from_result(self, result: Dict[str, Any], focused_id: str = None):
        self.ax.clear()
        self.scatter_to_id.clear(); self.id_to_points.clear()
        self.id_with_no_press.clear(); self.id_order.clear()
        self.focused_id = focused_id

        axl = result.get("axes_limits", {})
        fallback_xlim = tuple(axl.get("xlim",(0,100)))
        fallback_ylim = tuple(axl.get("ylim",(0,100)))
        fallback_zlim = tuple(axl.get("zlim",(0,100)))

        self.ax.set_xlabel("X [mm]"); self.ax.set_ylabel("Y [mm]"); self.ax.set_zlabel("Z [mm]")
        self.ax.grid(True)

        # --- IDごと
        by_id = {}
        for it in result.get("results", []):
            by_id.setdefault(str(it.get("id","")), []).append(it)
        sorted_ids = sorted(by_id.keys(), key=self._id_numeric_key)

        # IDリスト更新
        self.listbox.delete(0, 'end')
        for rid in sorted_ids:
            if rid:
                self.listbox.insert('end', rid)

        xs_all, ys_all, zs_all = [], [], []

        def sty(is_focus, base_color, s=36):
            if is_focus:
                return dict(c=base_color, edgecolors="k", alpha=1.0, s=s)
            else:
                return dict(c="#BBBBBB", edgecolors="#888888", alpha=0.25, s=max(18, int(s*0.7)))

        for rid in sorted_ids:
            it_list = by_id[rid]
            is_focus = (focused_id is None) or _ids_equal(rid, focused_id)
            for it in it_list:
                comp = it["computed"]

                if self.show_3.get():  # 実TCP
                    x,y,z = comp["actual_tcp"]
                    sargs = sty(is_focus, "r", s=36)
                    s = self.ax.scatter([x],[y],[z], marker="^", **sargs)
                    self.scatter_to_id[s] = rid
                    xs_all.append(x); ys_all.append(y); zs_all.append(z)

                if self.show_1.get():  # 目標TCP
                    x,y,z = comp["target_tcp"]
                    sargs = sty(is_focus, "b", s=30)
                    s = self.ax.scatter([x],[y],[z], marker="^", **sargs)
                    self.scatter_to_id[s] = rid
                    xs_all.append(x); ys_all.append(y); zs_all.append(z)

                if self.show_4.get() and comp["pressed"]:  # 押し込み
                    pts = [p for _, p in comp["pressed"]]
                    x = [p[0] for p in pts]; y=[p[1] for p in pts]; z=[p[2] for p in pts]
                    sargs = sty(is_focus, "r", s=18)
                    s = self.ax.scatter(x, y, z, marker="o", **sargs)
                    self.scatter_to_id[s] = rid
                    xs_all += x; ys_all += y; zs_all += z

                if self.show_5.get() and comp["unpressed"]:  # 未押
                    pts = [p for _, p in comp["unpressed"]]
                    x = [p[0] for p in pts]; y=[p[1] for p in pts]; z=[p[2] for p in pts]
                    sargs = sty(is_focus, "r", s=18)
                    s = self.ax.scatter(x, y, z, marker="x", c=sargs["c"], alpha=sargs["alpha"], s=sargs["s"])
                    self.scatter_to_id[s] = rid
                    xs_all += x; ys_all += y; zs_all += z

        # ズーム
        if self.autoscale.get():
            if xs_all and ys_all and zs_all:
                xlim, ylim, zlim = _compute_auto_limits(xs_all, ys_all, zs_all)
            else:
                xlim, ylim, zlim = fallback_xlim, fallback_ylim, fallback_zlim
        else:
            xlim = self.fixed_xlim or fallback_xlim
            ylim = self.fixed_ylim or fallback_ylim
            zlim = self.fixed_zlim or fallback_zlim
        self.ax.set_xlim(*xlim); self.ax.set_ylim(*ylim); self.ax.set_zlim(*zlim)

        # シミュ重ね描画（選択IDのみ色／他はグレー）※判定はSimOverlay側で_ids_equalを用いて行う
        if self.sim_overlay.enabled.get() and self.sim_overlay.sim is not None:
            self.sim_overlay.plot_sim_overlay(self.ax, focused_id)

        self._force_draw()
        # ※ 統計はここで計算しない（ボタン押下時のみ）




    # --- イベント ---
    def on_list_select(self, event):
        sel = self.listbox.curselection()
        if not sel:
            self.reset_focus()
            return
        rid = self.listbox.get(sel[0])
        if rid:
            self.focused_id = rid
            if self.current_result:
                self._plot_from_result(self.current_result, focused_id=rid)
                self.status.set(f"ID={rid} にフォーカス．（シミュ比較・統計もID限定）")


    def on_pick(self, event):
        rid = self.scatter_to_id.get(event.artist, None)
        if not rid: return
        try:
            idx = self.id_order.index(rid)
        except ValueError:
            return
        self.listbox.selection_clear(0, tk.END)
        self.listbox.selection_set(idx)
        self.listbox.see(idx)
        self.focused_id = rid
        self._plot_from_result(self.current_result, focused_id=rid)
        self.status.set(f"点クリック：ID {rid} をフォーカス．")

    def save_result(self):
        try:
            if not self.current_result:
                messagebox.showwarning("警告","まず『読込＆描画』でデータを読み込んでください．")
                return
            path = filedialog.asksaveasfilename(defaultextension=".json", filetypes=[("JSON","*.json")], title="結果ファイルの保存先")
            if not path: return
            with open(path,"w",encoding="utf-8") as f:
                json.dump(self.current_result, f, ensure_ascii=False, indent=2)
            self.status.set(f"結果ファイルを保存：{path}")
            log.info(f"保存: {path}")
        except Exception as e:
            messagebox.showerror("エラー", f"保存中にエラー：\n{e}")
            log.exception("保存例外")

    def _init_sim_widgets(self, parent):
        # ファイル選択
        frm_file = ttk.LabelFrame(parent, text="シミュレーションJSON")
        frm_file.grid(row=20, column=0, sticky="ew", pady=6)
        self.path_sim = tk.StringVar(value="")
        row = ttk.Frame(frm_file); row.grid(sticky="ew", pady=2); row.grid_columnconfigure(1, weight=1)
        ttk.Label(row, text="sim_result.json", width=24).grid(row=0, column=0, sticky="w")
        ttk.Entry(row, textvariable=self.path_sim).grid(row=0, column=1, sticky="ew", padx=4)
        ttk.Button(row, text="参照", command=self._browse_sim).grid(row=0, column=2, padx=2)

        # 有効化/表示種別
        frm_opt = ttk.LabelFrame(parent, text="シミュの描画対象")
        frm_opt.grid(row=21, column=0, sticky="ew", pady=6)
        ttk.Checkbutton(frm_opt, text="有効化", variable=self.sim_overlay.enabled, command=self.redraw_from_cache).grid(row=0, column=0, sticky="w", padx=4)
        ttk.Checkbutton(frm_opt, text="A(TCP▲)", variable=self.sim_overlay.show_A, command=self.redraw_from_cache).grid(row=0, column=1, sticky="w")
        ttk.Checkbutton(frm_opt, text="B(TCP▲)", variable=self.sim_overlay.show_B, command=self.redraw_from_cache).grid(row=0, column=2, sticky="w")
        ttk.Checkbutton(frm_opt, text="Pa(・)",   variable=self.sim_overlay.show_Pa, command=self.redraw_from_cache).grid(row=1, column=1, sticky="w")
        ttk.Checkbutton(frm_opt, text="Pb(・)",   variable=self.sim_overlay.show_Pb, command=self.redraw_from_cache).grid(row=1, column=2, sticky="w")
        ttk.Checkbutton(frm_opt, text="接触(○)",  variable=self.sim_overlay.show_contact, command=self.redraw_from_cache).grid(row=2, column=1, sticky="w")
        ttk.Checkbutton(frm_opt, text="非接触(×)",variable=self.sim_overlay.show_noncontact, command=self.redraw_from_cache).grid(row=2, column=2, sticky="w")

        # オフセット
        frm_off = ttk.LabelFrame(parent, text="シミュの座標オフセット [mm]（基準はログ側）")
        frm_off.grid(row=22, column=0, sticky="ew", pady=6)
        ttk.Label(frm_off, text="dx").grid(row=0, column=0, sticky="w"); 
        ttk.Spinbox(frm_off, textvariable=self.sim_overlay.offx, from_=-1000, to=1000, increment=0.5, width=10).grid(row=0, column=1)
        ttk.Label(frm_off, text="dy").grid(row=0, column=2, sticky="w"); 
        ttk.Spinbox(frm_off, textvariable=self.sim_overlay.offy, from_=-1000, to=1000, increment=0.5, width=10).grid(row=0, column=3)
        ttk.Label(frm_off, text="dz").grid(row=0, column=4, sticky="w"); 
        ttk.Spinbox(frm_off, textvariable=self.sim_overlay.offz, from_=-1000, to=1000, increment=0.5, width=10).grid(row=0, column=5)
        ttk.Button(frm_off, text="適用", command=self.redraw_from_cache).grid(row=0, column=6, padx=4)

    def _browse_sim(self):
        path = filedialog.askopenfilename(title="シミュレーションJSON", filetypes=[("JSON","*.json"),("All","*.*")])
        if path:
            self.path_sim.set(path)
            self.sim_overlay.load_sim(path)
            self.redraw_from_cache()

    def on_click_compute_stats(self):
        # 選択ID（なければ全体）に対して統計を計算
        fid = getattr(self, "focused_id", None)
        self._compute_and_show_stats(fid)

    
    def _compute_and_show_stats(self, focused_id: str | None):
        if not hasattr(self, "stats_var"):
            return
        if self.sim_overlay.sim is None or self.current_result is None:
            self.stats_var.set("統計: シミュ or 実機データが未読込")
            self._set_mismatch_lines([])
            return

        # 実機: id -> pin -> (point, contact)
        real_map = {}
        for it in self.current_result.get("results", []):
            rid = it.get("id", "")
            if focused_id and not _ids_equal(rid, focused_id):
                continue
            comp = it.get("computed")
            if not comp:
                continue
            R = real_map.setdefault(str(rid), {})
            for pin_idx, pt in comp["pressed"]:
                R[int(pin_idx)] = (pt, True)
            for pin_idx, pt in comp["unpressed"]:
                R[int(pin_idx)] = (pt, False)

        # シミュ: id -> pin -> (point(+off), contact)
        sim_map = {}
        for block in self.sim_overlay.sim.get("plan", []):
            rid = block.get("id")
            if focused_id and not _ids_equal(rid, focused_id):
                continue
            S = sim_map.setdefault(str(rid), {})
            for r in block.get("results", []):
                pin = int(r["pin"])
                pt  = self.sim_overlay._apply_off(r["point"])
                S[pin] = (pt, bool(r.get("contact", False)))

        # 比較
        diffs, adx, ady, adz = [], [], [], []
        mismatches = []
        cnt_total = cnt_fn = cnt_fp = 0

        for rid, pins in sim_map.items():
            # 対応id（正規化一致）を検索
            real_rid = None
            for rrid in real_map.keys():
                if _ids_equal(rrid, rid):
                    real_rid = rrid
                    break
            if real_rid is None:
                continue

            for pin, (sp, sc) in pins.items():
                if pin not in real_map[real_rid]:
                    continue
                rp, rc = real_map[real_rid][pin]
                dx = sp[0] - rp[0]; dy = sp[1] - rp[1]; dz = sp[2] - rp[2]
                d = math.sqrt(dx*dx + dy*dy + dz*dz)
                diffs.append(d); adx.append(abs(dx)); ady.append(abs(dy)); adz.append(abs(dz))
                if sc != rc:
                    cnt_total += 1
                    if (not sc) and rc: cnt_fn += 1    # simu×, real〇
                    elif sc and (not rc): cnt_fp += 1 # simu〇, real×
                    mismatches.append(f"id{rid}-p{pin} | simu:{'〇' if sc else '×'}, real:{'〇' if rc else '×'}")

        scope = f"ID={focused_id}" if focused_id else "全体"
        if diffs:
            mn = min(diffs); mx = max(diffs); av = sum(diffs)/len(diffs)
            def stats(arr): return (min(arr), max(arr), sum(arr)/len(arr)) if arr else (float('nan'),)*3
            sx, sy, sz = stats(adx), stats(ady), stats(adz)
            self.stats_var.set(
                "統計({scope})  距離誤差[min,max,mean]=[{mn:.3f},{mx:.3f},{av:.3f}] mm  件数={n}  "
                "|dx|=[{x0:.3f},{x1:.3f},{x2:.3f}] | "
                "|dy|=[{y0:.3f},{y1:.3f},{y2:.3f}] | "
                "|dz|=[{z0:.3f},{z1:.3f},{z2:.3f}]  "
                "不一致: {tot}件 (sim×,real〇={fn} / simu〇,real×={fp})"
                .format(scope=scope, mn=mn, mx=mx, av=av, n=len(diffs),
                        x0=sx[0], x1=sx[1], x2=sx[2],
                        y0=sy[0], y1=sy[1], y2=sy[2],
                        z0=sz[0], z1=sz[1], z2=sz[2],
                        tot=cnt_total, fn=cnt_fn, fp=cnt_fp)
            )
        else:
            self.stats_var.set(f"統計({scope}): 比較対象の点がありません")

        # 詳細はスクロールTextへ
        self._set_mismatch_lines(mismatches)


    def _set_mismatch_lines(self, lines):
        """接触不一致の詳細（idX-pY ...）を1行ずつTextへ出力する．"""
        if not hasattr(self, "mismatch_text"):
            return
        self.mismatch_text.configure(state="normal")
        self.mismatch_text.delete("1.0", "end")
        if lines:
            self.mismatch_text.insert("end", "\n".join(lines))
        self.mismatch_text.configure(state="disabled")



# 追加: シミュレーション重ね描画ヘルパ
class SimOverlay:
    def __init__(self, owner: "PinArtViewer"):
        self.owner = owner
        self.sim = None  # 読み込んだシミュJSON(dict)

        # 有効化と描画対象
        self.enabled = tk.BooleanVar(value=True)
        self.show_A = tk.BooleanVar(value=False)   # A TCP
        self.show_B = tk.BooleanVar(value=False)   # B TCP
        self.show_Pa = tk.BooleanVar(value=False)  # Pa格子
        self.show_Pb = tk.BooleanVar(value=False)  # Pb格子
        self.show_contact = tk.BooleanVar(value=True)   # results: contact True (○)
        self.show_noncontact = tk.BooleanVar(value=True)# results: contact False (×)

        # オフセット（mm）
        self.offx = tk.DoubleVar(value=0.0)
        self.offy = tk.DoubleVar(value=0.0)
        self.offz = tk.DoubleVar(value=0.0)

    def load_sim(self, path: str):
        if not path:
            self.sim = None
            return
        self.sim = load_json(path)

    def _apply_off(self, p):
        return [p[0] + self.offx.get(), p[1] + self.offy.get(), p[2] + self.offz.get()]

    def _iter_ids(self):
        if not self.sim:
            return []
        return [str(block.get("id")) for block in self.sim.get("plan", [])]

    def _iter_points_for_id(self, rid: str):
        """ridのブロックから，描画対象に応じた点群をyield"""
        if not self.sim:
            return
        for block in self.sim.get("plan", []):
            if str(block.get("id")) != str(rid):
                continue

            # A/B TCP
            if self.show_A.get() and "A" in block and len(block["A"]) >= 3:
                yield ("sim_A", [self._apply_off(block["A"][:3])])
            if self.show_B.get() and "B" in block and len(block["B"]) >= 3:
                yield ("sim_B", [self._apply_off(block["B"][:3])])

            # Pa/Pb グリッド
            if self.show_Pa.get() and "Pa" in block:
                yield ("sim_Pa", [self._apply_off(p) for p in block["Pa"]])
            if self.show_Pb.get() and "Pb" in block:
                yield ("sim_Pb", [self._apply_off(p) for p in block["Pb"]])

            # results（ピン先端 contact別）
            if "results" in block:
                if self.show_contact.get():
                    pts = [self._apply_off(r["point"]) for r in block["results"] if r.get("contact")]
                    if pts:
                        yield ("sim_contact", pts)
                if self.show_noncontact.get():
                    pts = [self._apply_off(r["point"]) for r in block["results"] if not r.get("contact")]
                    if pts:
                        yield ("sim_noncontact", pts)

    def plot_sim_overlay(self, ax, focused_id: str | None):
        ids = self._iter_ids()
        for rid in ids:
            is_focus = (focused_id is None) or _ids_equal(rid, focused_id)

            col = "g" if is_focus else "#6e6e6e"
            alpha = 1.0 if is_focus else 0.25
            size_p = 18 if is_focus else 14
            size_tcp = 34 if is_focus else 26
            edge = "k" if is_focus else "#888888"

            for kind, pts in self._iter_points_for_id(rid):
                if not pts:
                    continue
                xs = [p[0] for p in pts]; ys = [p[1] for p in pts]; zs = [p[2] for p in pts]

                if kind in ("sim_A","sim_B"):
                    ax.scatter(xs, ys, zs, marker="^", s=size_tcp, c=col, edgecolors=edge, alpha=alpha)
                elif kind in ("sim_Pa","sim_Pb"):
                    ax.scatter(xs, ys, zs, marker=".", s=12 if not is_focus else 14, c=col, alpha=alpha)
                elif kind == "sim_contact":
                    ax.scatter(xs, ys, zs, marker="o", s=size_p, c=col, edgecolors=edge, alpha=alpha)
                elif kind == "sim_noncontact":
                    ax.scatter(xs, ys, zs, marker="x", s=size_p, c=col, alpha=alpha)

def main():
    log.info(f"Python: {sys.version.split()[0]}  /  cwd: {os.getcwd()}")
    app = PinArtViewer()
    app.mainloop()

if __name__ == "__main__":
    main()
