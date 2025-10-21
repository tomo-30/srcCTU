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
    bases = pin_info["pin_bases_wrist_xyz_mm"]
    full  = float(pin_info["pin_length_mm"])
    tgt = entry["target"]["xyzrpy"]; act = entry["actual"]["xyzrpy"]; pins = entry["pins"]

    tgt_full = [transform_point(tgt, [b[0],b[1],b[2]+full]) for b in bases]
    act_pressed, act_unpressed = [], []
    for i,b in enumerate(bases):
        if i < len(pins) and pins[i] is not None and pins[i] >= 0:
            L = float(pins[i]); act_pressed.append(transform_point(act,[b[0],b[1],b[2]+L]))
        else:
            act_unpressed.append(transform_point(act,[b[0],b[1],b[2]+full]))
    return {
        "target_tcp": tgt[:3],
        "target_pin_tips_full": tgt_full,
        "actual_tcp": act[:3],
        "actual_pin_tips_pressed": act_pressed,
        "actual_pin_tips_unpressed": act_unpressed,
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
            if e.get("type") == "pin_trigger": n_pintrigger += 1
    log.info(f"ログ: 全{len(log_data.get('logs',[]))}件 / 対象{len(results)}件 / pin_trigger {n_pintrigger}件")
    return {"meta":{"version":"1.9"}, "axes_limits":axes_limits, "results":results}

# ===== GUI本体 =====
class PinArtViewer(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Pin-Art Sensor Point Cloud Viewer (rev9)")
        self.geometry("1360x820")

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
        self.id_order: List[str] = []
        self.id_to_points: Dict[str, List[Tuple[float,float,float]]] = {}
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
        # 追加: OFF時に保持する固定範囲（Noneなら未確定）
        self.fixed_xlim = None
        self.fixed_ylim = None
        self.fixed_zlim = None
        frm_zoom = ttk.LabelFrame(left, text="表示範囲")
        frm_zoom.grid(row=12, column=0, sticky="ew", pady=6)
        ttk.Checkbutton(
            frm_zoom,
            text="自動ズーム（描画要素に合わせる）",
            variable=self.autoscale,
            command=self.on_toggle_autoscale
        ).grid(row=0, column=0, sticky="w", padx=4)

        frm_btn = ttk.Frame(left)
        frm_btn.grid(row=11, column=0, sticky="ew", pady=6)
        ttk.Button(frm_btn, text="読込＆描画", command=self.load_and_plot).grid(row=0, column=0, padx=3)
        ttk.Button(frm_btn, text="結果ファイルとして保存", command=self.save_result).grid(row=0, column=1, padx=3)
        ttk.Button(frm_btn, text="クリア", command=self._clear_plot).grid(row=0, column=2, padx=3)

        # === 右ペイン ===
        right = ttk.Frame(self, padding=6)
        right.grid(row=0, column=2, sticky="ns")

        frm_top = ttk.Frame(right)
        frm_top.grid(row=0, column=0, sticky="ew")
        ttk.Label(frm_top, text="目標値ID（クリックでフォーカス）").pack(side=tk.LEFT)
        ttk.Button(frm_top, text="クリア", command=self.reset_focus).pack(side=tk.RIGHT, padx=4)

        self.listbox = tk.Listbox(right, height=28)
        self.listbox.grid(row=1, column=0, sticky="ns")
        self.listbox.bind("<<ListboxSelect>>", self.on_list_select)

        self.counter_var = tk.StringVar(value="押し込み回数: 0 ／ 点群総数(赤○): 0 ／ 描画点数: 0")
        ttk.Label(right, textvariable=self.counter_var).grid(row=2, column=0, sticky="w", pady=6)

        # === 中央ペイン（キャンバス） ===
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
            if self.path_result.get():
                result = load_json(self.path_result.get())
                log.info("結果ファイルから読込")
            else:
                if not (self.path_log.get() and self.path_push.get() and self.path_pininfo.get()):
                    messagebox.showwarning("警告","log / push_plan / pinart_info を指定してください．")
                    return
                log_data = load_json(self.path_log.get())
                pin_info = load_json(self.path_pininfo.get())
                push_plan = load_json(self.path_push.get())
                xlim,ylim,zlim = compute_axes_from_pushplan(push_plan)
                result = make_complete_result(log_data, pin_info, {"xlim":xlim,"ylim":ylim,"zlim":zlim})
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

        # --- 従来の固定範囲（フォールバック用に保持）
        axl = result.get("axes_limits", {})
        fallback_xlim = tuple(axl.get("xlim",(0,100)))
        fallback_ylim = tuple(axl.get("ylim",(0,100)))
        fallback_zlim = tuple(axl.get("zlim",(0,100)))

        self.ax.set_xlabel("X [mm]"); self.ax.set_ylabel("Y [mm]"); self.ax.set_zlabel("Z [mm]")
        self.ax.grid(True)

        # IDでグループ化
        by_id={}
        for it in result.get("results",[]):
            by_id.setdefault(it.get("id",""),[]).append(it)
        sorted_ids = sorted(by_id.keys(), key=self._id_numeric_key)

        s1=s2=s3=s4=s5=None
        drawn_points = 0
        press_id_count = 0

        # ★ ここで描画される全点を回収しておく（自動ズーム用）
        xs_all, ys_all, zs_all = [], [], []

        for rid in sorted_ids:
            if focused_id and rid != focused_id:
                clr_red = "lightgray"; edge = "gray"; a = 0.25
                clr_blue = "lightgray"
            else:
                clr_red = "r"; edge = "k"; a = 0.9
                clr_blue = "b"

            entries = by_id[rid]
            pressed_pts_for_id=[]
            for item in entries:
                comp = item.get("computed",{})

                if self.show_1.get():
                    x,y,z = comp.get("target_tcp",[None,None,None])
                    if x is not None:
                        s1 = self.ax.scatter([x],[y],[z], marker="^", s=40, edgecolors=edge, c=clr_blue, alpha=a)
                        drawn_points += 1
                        if not (focused_id and rid != focused_id):
                            xs_all.append(x); ys_all.append(y); zs_all.append(z)

                if self.show_2.get():
                    pts = comp.get("target_pin_tips_full",[])
                    if pts:
                        xs,ys,zs = zip(*pts)
                        s2 = self.ax.scatter(xs,ys,zs, marker="o", s=20, edgecolors=edge, c=clr_blue, alpha=a)
                        drawn_points += len(xs)
                        if not (focused_id and rid != focused_id):
                            xs_all.extend(xs); ys_all.extend(ys); zs_all.extend(zs)

                if self.show_3.get():
                    x,y,z = comp.get("actual_tcp",[None,None,None])
                    if x is not None:
                        s3 = self.ax.scatter([x],[y],[z], marker="^", s=40, edgecolors=edge, c=clr_red, alpha=a)
                        drawn_points += 1
                        if not (focused_id and rid != focused_id):
                            xs_all.append(x); ys_all.append(y); zs_all.append(z)

                if self.show_4.get():
                    pts = comp.get("actual_pin_tips_pressed",[])
                    if pts:
                        xs,ys,zs = zip(*pts)
                        sc = self.ax.scatter(xs,ys,zs, marker="o", s=26, edgecolors=edge, c=clr_red, alpha=a, picker=True, pickradius=5)
                        self.scatter_to_id[sc]=rid
                        self.id_to_points.setdefault(rid,[]).extend(pts)
                        pressed_pts_for_id.extend(pts)
                        s4=sc; drawn_points += len(xs)
                        if not (focused_id and rid != focused_id):
                            xs_all.extend(xs); ys_all.extend(ys); zs_all.extend(zs)

                if self.show_5.get():
                    pts = comp.get("actual_pin_tips_unpressed",[])
                    if pts:
                        xs,ys,zs = zip(*pts)
                        s5 = self.ax.scatter(xs,ys,zs, marker="x", s=42, c=clr_red, alpha=a)
                        drawn_points += len(xs)
                        if not (focused_id and rid != focused_id):
                            xs_all.extend(xs); ys_all.extend(ys); zs_all.extend(zs)

            if len(pressed_pts_for_id)==0:
                self.id_with_no_press.add(rid)
            else:
                press_id_count += 1
            self.id_order.append(rid)

        # --- 自動ズーム or フォールバック/固定範囲設定
        if self.autoscale.get():
            # 自動：描画対象に合わせて都度再計算
            if xs_all and ys_all and zs_all:
                xlim, ylim, zlim = _compute_auto_limits(xs_all, ys_all, zs_all)
            else:
                # 描画対象がなくても破綻しないようにフォールバック
                xlim, ylim, zlim = fallback_xlim, fallback_ylim, fallback_zlim

            # 自動ズームの現在値は「最新の調整結果」なので、
            # このあとOFFに切り替えたら on_toggle_autoscale() で保持される
        else:
            # 固定：最後にOFFへ切り替えた瞬間に保存した値を常に使う
            # （未設定ならフォールバック）
            xlim = self.fixed_xlim or fallback_xlim
            ylim = self.fixed_ylim or fallback_ylim
            zlim = self.fixed_zlim or fallback_zlim

        self.ax.set_xlim(*xlim); self.ax.set_ylim(*ylim); self.ax.set_zlim(*zlim)

        # 等尺比
        try:
            self.ax.set_box_aspect((xlim[1]-xlim[0], ylim[1]-ylim[0], zlim[1]-zlim[0]))
        except Exception:
            pass

        # --- 以降、凡例やタイトルなどは従来どおり
        self.listbox.delete(0, tk.END)
        for rid in self.id_order:
            self.listbox.insert(tk.END, rid or "(no-id)")
            idx = self.listbox.size()-1
            if rid in self.id_with_no_press:
                try: self.listbox.itemconfig(idx, foreground="blue")
                except Exception: pass

        total_points = sum(len(self.id_to_points.get(rid,[])) for rid in self.id_order)
        self.counter_var.set(f"押し込み回数: {press_id_count} ／ 点群総数(赤○): {total_points} ／ 描画点数: {drawn_points}")

        handles,labels=[],[]
        if s1: handles.append(s1); labels.append("① 目標TCP")
        if s2: handles.append(s2); labels.append("② 目標ピンfull")
        if s3: handles.append(s3); labels.append("③ 実TCP")
        if s4: handles.append(s4); labels.append("④ 実ピン押込")
        if s5: handles.append(s5); labels.append("⑤ 実ピン未押")
        if handles: self.ax.legend(handles, labels, loc="upper right")

        self.ax.set_title("Pin-Art Sensor Point Cloud" + (f"  [Focus: {focused_id}]" if focused_id else ""))
        self.ax.view_init(elev=20, azim=-60)
        self._force_draw()

        if drawn_points==0:
            self.status.set("描画対象が0点です．")
            log.warning("描画点数が0")
        else:
            self.status.set(f"描画完了．総点数 {drawn_points} 点．")


    # --- イベント ---
    def on_list_select(self, event):
        sel = self.listbox.curselection()
        if not sel: return
        rid = self.id_order[sel[0]] if sel[0] < len(self.id_order) else ""
        self.focused_id = rid
        self._plot_from_result(self.current_result, focused_id=rid)
        self.status.set(f"選択ID: {rid} のみを強調表示．")

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

def main():
    log.info(f"Python: {sys.version.split()[0]}  /  cwd: {os.getcwd()}")
    app = PinArtViewer()
    app.mainloop()

if __name__ == "__main__":
    main()
