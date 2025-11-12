# ellipsoid_viewer.py
# -*- coding: utf-8 -*-
"""
楕円体フィッティング可視化GUI（Tkinter + Matplotlib 3D）
- ○（接触点）＋ ×（同一(x,y)から+Z方向は空）を同時に使用して楕円体を推定
- 地面 z = z_ground に絶対にめり込まないよう制約（下端=min_z>=z_ground）
- シミュ（青）／実機（赤）の表示ON/OFF，IDクリックで選択トグル＆クリア
- 可視点（= 色がついている点）に対して，SIM→緑，REAL→黄の楕円体を描画
- 「地面高さ」をGUIで設定可能（既定 z=0）
- ×点は「楕円体の外側」かつ「同一(x,y)起点の+Z縦レイ非交差」を満たすよう拘束
- 実機の3x3ピン再構成はGUIで dx,dy,Z0,k を調整可能

依存: Python3.9+, numpy, matplotlib, tkinter, json
"""

import os, json, math
from typing import Dict, List, Tuple, Optional, Set
import numpy as np

import tkinter as tk
from tkinter import ttk, filedialog, messagebox

from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt


# =========================
# ユーティリティ
# =========================
def rot_to_rpy_ZYX(R):
    import math, numpy as np
    sy = -R[2, 0]
    cy = math.sqrt(max(0.0, 1.0 - sy * sy))
    if cy > 1e-8:
        yaw = math.degrees(math.atan2(R[1, 0], R[0, 0]))
        pitch = math.degrees(math.asin(sy))
        roll = math.degrees(math.atan2(R[2, 1], R[2, 2]))
    else:
        yaw = math.degrees(math.atan2(-R[0, 1], R[1, 1]))
        pitch = math.degrees(math.asin(sy))
        roll = 0.0
    return roll, pitch, yaw


def is_sim_file(path: str) -> bool:
    try:
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
        return "plan" in data and isinstance(data["plan"], list)
    except Exception:
        return False


def is_real_log_file(path: str) -> bool:
    try:
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
        if "logs" not in data or not isinstance(data["logs"], list):
            return False
        return any((e.get("type") == "pin_trigger") for e in data["logs"])
    except Exception:
        return False


def pca_fit_from_contacts(points_xyz):
    import numpy as np, math
    if points_xyz is None or len(points_xyz) < 4:
        return None
    P = np.asarray(points_xyz, float)
    c = P.mean(axis=0)
    Q = P - c
    C = np.cov(Q.T) + 1e-9 * np.eye(3)
    lam, V = np.linalg.eigh(C)
    idx = np.argsort(lam)[::-1]
    lam = lam[idx]
    V = V[:, idx]
    return dict(center=c, R=V, lam=lam)


def make_ellipsoid_mesh(center, axes, R, nu=48, nv=24):
    import numpy as np
    u = np.linspace(0, 2*np.pi, nu)
    v = np.linspace(0, np.pi, nv)
    uu, vv = np.meshgrid(u, v)
    S = np.stack([np.cos(uu)*np.sin(vv), np.sin(uu)*np.sin(vv), np.cos(vv)], axis=-1)
    S = S * axes
    S = S @ R.T + center
    return S[...,0], S[...,1], S[...,2]



# 実機ログ 3x3 ピン配置（行=Y，列=X）
PIN_OFFSETS_RC = {
    1: (-1, -1), 2: (0, -1), 3: (1, -1),
    4: (-1,  0), 5: (0,  0), 6: (1,  0),
    7: (-1,  1), 8: (0,  1), 9: (1,  1),
}


# =========================
# 新フィッティング本体
# =========================
def _solve_scale_contact_strict_with_empty_soft(
    P_contact, P_empty, c_init, R, lam, z_ground,
    t_lo=1e-3, t_hi=1e3, iters=48,
    EPS_CONTACT=1e-6,  # ○内包の許容
    xy_search_rad=10.0, xy_steps=5      # XY微調整の半径[mm]と分割数
):
    """
    目的:
      1) ○: 全点が F(x)<=1+EPS_CONTACT となる最小スケール t を厳密に求める（厳密内包）
      2) 接地: min_z = z_ground を厳密に維持
      3) ×: 上記を壊さない範囲で (cx,cy) を微調整し, min_d2_empty_rays を最大化（軟約束）

    戻り:
      dict(center, axes, t, empty_metric, contact_ok=True)
    """
    import numpy as np, math
    P_contact = np.asarray(P_contact, float)
    P_empty   = np.asarray(P_empty,   float)

    lam = np.asarray(lam, float)
    rz = R[2, :]

    def axes_of(t):
        return np.sqrt(np.maximum(1e-12, (t*t)*lam))

    def center_of(t, cx, cy):
        a = axes_of(t)
        Lz = math.sqrt(max(1e-12, (a[0]*rz[0])**2 + (a[1]*rz[1])**2 + (a[2]*rz[2])**2))
        c = np.array([cx, cy, z_ground + Lz], float)
        return c, a

    # d^2 for contacts (max)
    def max_d2_contacts(t, cx, cy):
        if not P_contact.size: return 0.0
        c,a = center_of(t, cx, cy)
        Q = (P_contact - c) @ R
        inv2 = 1.0/np.maximum(1e-12, a*a)
        d2 = np.sum(Q*(Q*inv2), axis=1)
        return float(np.max(d2))

    # min d^2 over +Z rays from empty points
    ez = np.array([0.0,0.0,1.0], float)
    def min_d2_empty_rays(t, cx, cy):
        if not P_empty.size: return float("+inf")
        c,a = center_of(t, cx, cy)
        A = R @ np.diag(1.0/np.maximum(1e-12, a*a)) @ R.T
        alpha = float(ez.T @ A @ ez)
        vals=[]
        for p in P_empty:
            u = p - c
            beta  = float(u.T @ A @ ez)
            gamma = float(u.T @ A @ u)
            if alpha <= 1e-18:
                fmin = gamma
            else:
                s_star = max(0.0, -beta/alpha)
                fmin = alpha*s_star*s_star + 2.0*beta*s_star + gamma
            vals.append(fmin)
        return float(np.min(vals))

    # 1) ○を全内包する最小 t（max_d2<=1+EPS）を二分探索
    cx0, cy0 = float(c_init[0]), float(c_init[1])
    lo, hi = t_lo, t_hi
    for _ in range(iters):
        mid = math.sqrt(lo*hi)
        if max_d2_contacts(mid, cx0, cy0) > 1.0 + EPS_CONTACT:
            lo = mid
        else:
            hi = mid
    t_star = math.sqrt(lo*hi)

    # 2) ×に対して (cx,cy) を微調整（tは固定）
    #    接地は center_of() が常に満たす．○を再チェックしながら min_d2_empty を最大化
    best = {"cx": cx0, "cy": cy0,
            "metric": min_d2_empty_rays(t_star, cx0, cy0)}
    if xy_search_rad > 1e-9 and P_empty.size:
        # 等間隔グリッド + 単純局所探索（軽量）
        for it in range(2):  # 2層程度で十分軽い
            cand = []
            rad = xy_search_rad / (it+1)
            steps = max(2, xy_steps)
            for ix in range(-steps, steps+1):
                for iy in range(-steps, steps+1):
                    cx = best["cx"] + rad * ix/steps
                    cy = best["cy"] + rad * iy/steps
                    # ○厳密条件を守れているか
                    if max_d2_contacts(t_star, cx, cy) > 1.0 + EPS_CONTACT:
                        continue
                    m = min_d2_empty_rays(t_star, cx, cy)
                    cand.append((m, cx, cy))
            if cand:
                m, cx, cy = max(cand, key=lambda x:x[0])
                if m > best["metric"]:
                    best = {"cx": cx, "cy": cy, "metric": m}

    c_fin, a_fin = center_of(t_star, best["cx"], best["cy"])
    return dict(center=c_fin, axes=a_fin, t=t_star,
                empty_metric=best["metric"], contact_ok=True)

def fit_ellipsoid_with_empty_and_ground(
    points_with_flags,
    z_ground: float = 0.0,
    w_sphere: float = 0.3,
    ratio_cap: float = 1.8,
    alpha_align_z: float = 0.5,
    EPS_CONTACT: float = 1e-6,
    xy_search_rad: float = 10.0,
    xy_steps: int = 5,
):
    """
    仕様変更:
      - ○/接地は厳密．×は"できる限り"（XY微調整で最小侵入）
      - 分位指定(quantile)とdelta_emptyは廃止（不要）
    """
    import numpy as np, math
    if not points_with_flags:
        return None

    P = np.asarray([(x,y,z) for (x,y,z,_) in points_with_flags], float)
    Cmask = np.asarray([bool(c) for (*_,c) in points_with_flags], bool)
    P_contact = P[Cmask];  P_empty = P[~Cmask]
    if P_contact.shape[0] < 4:
        return None

    base = pca_fit_from_contacts(P_contact)
    if base is None: return None
    c0, R, lam = base["center"], base["R"], base["lam"]

    # 姿勢ロバスト化（既存と同様）
    if alpha_align_z > 0.0:
        ez = np.array([0.0,0.0,1.0]); v3 = R[:,2]
        v3b = (1-alpha_align_z)*v3 + alpha_align_z*ez; v3b/=max(1e-12, np.linalg.norm(v3b))
        v1  = R[:,0]; v1/=max(1e-12, np.linalg.norm(v1))
        v2  = np.cross(v3b, v1); n2=np.linalg.norm(v2)
        if n2 < 1e-9:
            v1 = np.array([1,0,0], float); v2=np.cross(v3b, v1); v2/=np.linalg.norm(v2)
        else:
            v2/=n2
        v1 = np.cross(v2, v3b); R = np.stack([v1,v2,v3b], axis=1)

    if w_sphere > 0.0:
        m = float(np.mean(lam)); lam = (1-w_sphere)*lam + w_sphere*m
    if ratio_cap is not None and ratio_cap >= 1.0:
        i_max = int(np.argmax(lam)); i_min = int(np.argmin(lam))
        lam[i_min] = max(lam[i_min], 1e-12)
        ratio = math.sqrt(lam[i_max]/lam[i_min])
        if ratio > ratio_cap:
            lam[i_max] = (ratio_cap**2) * lam[i_min]

    sol = _solve_scale_contact_strict_with_empty_soft(
        P_contact=P_contact, P_empty=P_empty,
        c_init=c0, R=R, lam=lam, z_ground=z_ground,
        EPS_CONTACT=EPS_CONTACT,
        xy_search_rad=xy_search_rad, xy_steps=xy_steps
    )
    if sol is None: return None
    return dict(center=sol["center"], R=R, axes=sol["axes"], t=sol["t"],
                contact_ok=True,
                empty_metric=float(sol["empty_metric"]))

# =========================
# データ取り回し
# =========================
class DataStore:
    """
    保持形式:
      sim_points_by_id[id]  = [(x,y,z, contact_bool), ...]
      real_points_by_id[id] = [(x,y,z, contact_bool), ...]
    """
    def __init__(self):
        self.sim_points_by_id: Dict[str, List[Tuple[float,float,float,bool]]] = {}
        self.real_points_by_id: Dict[str, List[Tuple[float,float,float,bool]]] = {}

    def load_sim(self, path: str):
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
        self.sim_points_by_id.clear()
        for blk in data.get("plan", []):
            id_str = str(blk.get("id", "NA"))
            pts = []
            for res in blk.get("results", []):
                p = res.get("point")
                c = bool(res.get("contact", False))
                if p and len(p) == 3:
                    pts.append((float(p[0]), float(p[1]), float(p[2]), c))
            if pts:
                self.sim_points_by_id[id_str] = pts

    def load_real(self, path: str, dx: float, dy: float, z0: float, k_scale: float):
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
        self.real_points_by_id.clear()
        for e in data.get("logs", []):
            if e.get("type") != "pin_trigger":
                continue
            id_str = str(e.get("target", {}).get("id", "NA"))
            actual = e.get("actual", {})
            xyzrpy = actual.get("xyzrpy")
            if not xyzrpy or len(xyzrpy) < 3:
                continue
            Xc, Yc = float(xyzrpy[0]), float(xyzrpy[1])
            pins = e.get("pins", None)
            if not pins or len(pins) != 9:
                continue
            pts = []
            for idx, val in enumerate(pins, start=1):
                contact = (val is not None and float(val) > 0.0)
                r_off, c_off = PIN_OFFSETS_RC[idx]
                x = Xc + (c_off * dx)
                y = Yc + (r_off * dy)
                dz = max(0.0, float(val)) if (val is not None) else 0.0
                z = z0 + k_scale * dz
                pts.append((x, y, z, contact))
            if pts:
                self.real_points_by_id[id_str] = pts


# =========================
# GUI 本体
# =========================
class EllipsoidGUI(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Ellipsoid Fitting Viewer (hits + empty-rays, ground-constrained)")
        self.geometry("1480x880")

        self.data = DataStore()

        # 表示フラグ
        self.show_sim = tk.BooleanVar(value=True)
        self.show_real = tk.BooleanVar(value=True)

        # 地面高さ
        self.ground_z = tk.DoubleVar(value=0.0)  # 既定 z=0

        # 実機再構成パラメータ
        self.dx = tk.DoubleVar(value=30.0)
        self.dy = tk.DoubleVar(value=30.0)
        self.z0 = tk.DoubleVar(value=80.0)
        self.k_scale = tk.DoubleVar(value=1.0)

        # 選択ID
        self.selected_ids: Set[str] = set()

        self._build_ui()
        self._init_plot()

    # ---------- UI ----------
    def _build_ui(self):
        self.columnconfigure(0, weight=1)
        self.columnconfigure(1, weight=0)
        self.rowconfigure(0, weight=1)

        # 左：3D
        left = ttk.Frame(self)
        left.grid(row=0, column=0, sticky="nsew", padx=6, pady=6)
        left.rowconfigure(0, weight=1)
        left.rowconfigure(1, weight=0)
        left.columnconfigure(0, weight=1)

        self.fig = plt.figure(figsize=(9, 6.6))
        self.ax = self.fig.add_subplot(111, projection="3d")
        self.canvas = FigureCanvasTkAgg(self.fig, master=left)
        self.canvas.get_tk_widget().grid(row=0, column=0, sticky="nsew")

        toolbar_frame = ttk.Frame(left)
        toolbar_frame.grid(row=1, column=0, sticky="ew")
        self.toolbar = NavigationToolbar2Tk(self.canvas, toolbar_frame)
        self.toolbar.update()

        # 右：操作
        right = ttk.Frame(self)
        right.grid(row=0, column=1, sticky="ns", padx=6, pady=6)
        for r in range(12):
            right.rowconfigure(r, weight=0)
        right.rowconfigure(11, weight=1)
        right.columnconfigure(0, weight=1)

        frm_file = ttk.LabelFrame(right, text="ファイル選択")
        frm_file.grid(row=0, column=0, sticky="ew", pady=4)
        frm_file.columnconfigure(1, weight=1)
        ttk.Label(frm_file, text="シミュJSON:").grid(row=0, column=0, sticky="w")
        self.ent_sim = ttk.Entry(frm_file, width=38); self.ent_sim.grid(row=0, column=1, sticky="ew", padx=4)
        ttk.Button(frm_file, text="参照", command=self.on_browse_sim).grid(row=0, column=2)
        ttk.Label(frm_file, text="実機JSON:").grid(row=1, column=0, sticky="w")
        self.ent_real = ttk.Entry(frm_file, width=38); self.ent_real.grid(row=1, column=1, sticky="ew", padx=4)
        ttk.Button(frm_file, text="参照", command=self.on_browse_real).grid(row=1, column=2)
        ttk.Button(frm_file, text="読込/更新", command=self.on_load).grid(row=2, column=0, columnspan=3, sticky="ew", pady=4)

        frm_view = ttk.LabelFrame(right, text="表示切替")
        frm_view.grid(row=1, column=0, sticky="ew", pady=4)
        ttk.Checkbutton(frm_view, text="シミュ表示（青）", variable=self.show_sim, command=self.redraw).grid(row=0, column=0, sticky="w")
        ttk.Checkbutton(frm_view, text="実機表示（赤）", variable=self.show_real, command=self.redraw).grid(row=1, column=0, sticky="w")

        frm_ground = ttk.LabelFrame(right, text="地面制約")
        frm_ground.grid(row=2, column=0, sticky="ew", pady=4)
        ttk.Label(frm_ground, text="地面高さ z =").grid(row=0, column=0, sticky="e")
        ttk.Entry(frm_ground, textvariable=self.ground_z, width=9).grid(row=0, column=1, sticky="w")

        frm_real = ttk.LabelFrame(right, text="実機再構成設定")
        frm_real.grid(row=3, column=0, sticky="ew", pady=4)
        for i in range(4):
            frm_real.columnconfigure(i, weight=1)
        ttk.Label(frm_real, text="dx[mm]").grid(row=0, column=0, sticky="e")
        ttk.Entry(frm_real, textvariable=self.dx, width=8).grid(row=0, column=1, sticky="w")
        ttk.Label(frm_real, text="dy[mm]").grid(row=0, column=2, sticky="e")
        ttk.Entry(frm_real, textvariable=self.dy, width=8).grid(row=0, column=3, sticky="w")
        ttk.Label(frm_real, text="Z0[mm]").grid(row=1, column=0, sticky="e")
        ttk.Entry(frm_real, textvariable=self.z0, width=8).grid(row=1, column=1, sticky="w")
        ttk.Label(frm_real, text="k(押込→Z)").grid(row=1, column=2, sticky="e")
        ttk.Entry(frm_real, textvariable=self.k_scale, width=8).grid(row=1, column=3, sticky="w")

        frm_ids = ttk.LabelFrame(right, text="ID 選択（クリックでトグル）")
        frm_ids.grid(row=4, column=0, sticky="nsew", pady=4)
        frm_ids.rowconfigure(0, weight=1); frm_ids.columnconfigure(0, weight=1)
        self.lst_ids = tk.Listbox(frm_ids, selectmode=tk.SINGLE, height=12)
        self.lst_ids.grid(row=0, column=0, sticky="nsew")
        self.lst_ids.bind("<<ListboxSelect>>", self.on_id_click)
        sb = ttk.Scrollbar(frm_ids, orient="vertical", command=self.lst_ids.yview)
        sb.grid(row=0, column=1, sticky="ns")
        self.lst_ids.configure(yscrollcommand=sb.set)
        ttk.Button(frm_ids, text="クリア（全解除）", command=self.on_clear_selection).grid(row=1, column=0, columnspan=2, sticky="ew", pady=4)

        frm_fit = ttk.LabelFrame(right, text="楕円体フィッティング")
        frm_fit.grid(row=5, column=0, sticky="ew", pady=4)
        ttk.Button(frm_fit, text="可視点（○＋×）でフィット", command=self.on_fit).grid(row=0, column=0, sticky="ew")

        frm_info = ttk.LabelFrame(right, text="楕円体情報 / 整合性チェック")
        frm_info.grid(row=6, column=0, sticky="nsew", pady=4)
        frm_info.rowconfigure(0, weight=1); frm_info.columnconfigure(0, weight=1)
        self.txt_info = tk.Text(frm_info, height=14, wrap="word")
        self.txt_info.grid(row=0, column=0, sticky="nsew")
        sb2 = ttk.Scrollbar(frm_info, orient="vertical", command=self.txt_info.yview)
        sb2.grid(row=0, column=1, sticky="ns")
        self.txt_info.configure(yscrollcommand=sb2.set)

    def _init_plot(self):
        self.ax.cla()
        self.ax.set_xlabel("X [mm]")
        self.ax.set_ylabel("Y [mm]")
        self.ax.set_zlabel("Z [mm]")
        self.ax.view_init(elev=22, azim=-60)
        self.ax.grid(True)
        self._draw_ground()
        self.canvas.draw_idle()

    def _draw_ground(self):
        z = self.ground_z.get()
        lim = 200.0
        X, Y = np.meshgrid(np.linspace(-lim, lim, 2), np.linspace(-lim, lim, 2))
        Z = np.full_like(X, z)
        self.ground_surf = self.ax.plot_surface(X, Y, Z, alpha=0.08, edgecolor='none')

    # ---------- ファイル操作 ----------
    def on_browse_sim(self):
        p = filedialog.askopenfilename(title="シミュJSONを選択", filetypes=[("JSON", "*.json"), ("All", "*.*")])
        if p:
            self.ent_sim.delete(0, tk.END); self.ent_sim.insert(0, p)

    def on_browse_real(self):
        p = filedialog.askopenfilename(title="実機JSONを選択", filetypes=[("JSON", "*.json"), ("All", "*.*")])
        if p:
            self.ent_real.delete(0, tk.END); self.ent_real.insert(0, p)

    def on_load(self):
        sim_p = self.ent_sim.get().strip()
        real_p = self.ent_real.get().strip()

        loaded = False
        if sim_p:
            if not os.path.isfile(sim_p) or not is_sim_file(sim_p):
                messagebox.showwarning("警告", "シミュJSONの形式が想定と異なります．")
            else:
                self.data.load_sim(sim_p); loaded = True
        if real_p:
            if not os.path.isfile(real_p) or not is_real_log_file(real_p):
                messagebox.showwarning("警告", "実機JSONの形式が想定と異なります．")
            else:
                self.data.load_real(real_p, self.dx.get(), self.dy.get(), self.z0.get(), self.k_scale.get()); loaded = True

        if not loaded:
            messagebox.showinfo("情報", "読み込めるデータがありません．"); return

        self._refresh_id_list()
        self.redraw()

    def _refresh_id_list(self):
        all_ids = set(self.data.sim_points_by_id.keys()) | set(self.data.real_points_by_id.keys())
        ids_sorted = sorted(all_ids, key=lambda s: (len(s), s))
        self.lst_ids.delete(0, tk.END)
        for i in ids_sorted:
            self.lst_ids.insert(tk.END, i)

    # ---------- 選択 ----------
    def on_id_click(self, _):
        sel = self.lst_ids.curselection()
        if not sel: return
        id_str = self.lst_ids.get(sel[0])
        if id_str in self.selected_ids: self.selected_ids.remove(id_str)
        else: self.selected_ids.add(id_str)
        self.redraw()

    def on_clear_selection(self):
        self.selected_ids.clear()
        self.redraw()

    # ---------- 可視点収集（○/×分離） ----------
    def _gather_visible_points(self) -> Dict[str, Dict[str, np.ndarray]]:
        """
        戻り:
          {
            "sim": {"contact": Nx3, "noncontact": Mx3},
            "real":{"contact": Kx3, "noncontact": Lx3}
          }
        可視＝「表示ON かつ （未選択IDがあるなら）選択IDに一致」の点のみ．
        """
        res = {"sim": {"contact": [], "noncontact": []},
               "real": {"contact": [], "noncontact": []}}

        # SIM
        if self.show_sim.get():
            for id_str, pts in self.data.sim_points_by_id.items():
                colored = (not self.selected_ids) or (id_str in self.selected_ids)
                if not colored: continue
                for x, y, z, c in pts:
                    (res["sim"]["contact"] if c else res["sim"]["noncontact"]).append([x, y, z])
        # REAL
        if self.show_real.get():
            for id_str, pts in self.data.real_points_by_id.items():
                colored = (not self.selected_ids) or (id_str in self.selected_ids)
                if not colored: continue
                for x, y, z, c in pts:
                    (res["real"]["contact"] if c else res["real"]["noncontact"]).append([x, y, z])

        for k1 in res:
            for k2 in res[k1]:
                arr = np.array(res[k1][k2], float) if res[k1][k2] else np.empty((0, 3), float)
                res[k1][k2] = arr
        return res

    # ---------- 描画 ----------
    def redraw(self):
        self._init_plot()
        vis = self._gather_visible_points()

        def scatter_kind(arr_c: np.ndarray, arr_n: np.ndarray, color: str):
            if arr_c.size:
                self.ax.scatter(arr_c[:, 0], arr_c[:, 1], arr_c[:, 2], marker='o', s=32, alpha=0.95, color=color)
            if arr_n.size:
                self.ax.scatter(arr_n[:, 0], arr_n[:, 1], arr_n[:, 2], marker='x', s=32, alpha=0.95, color=color)

        scatter_kind(vis["sim"]["contact"],     vis["sim"]["noncontact"],     "blue")
        scatter_kind(vis["real"]["contact"],    vis["real"]["noncontact"],    "red")

        self._autoscale_equal(vis)
        self.canvas.draw_idle()

    def _autoscale_equal(self, vis):
        # 表示範囲算出
        any_pts = any(vis[k][t].size for k in vis for t in vis[k])
        all_pts = np.vstack([
            vis["sim"]["contact"], vis["sim"]["noncontact"],
            vis["real"]["contact"], vis["real"]["noncontact"]
        ]) if any_pts else np.array([[0,0,0]])
        xmin, ymin, zmin = np.min(all_pts, axis=0)
        xmax, ymax, zmax = np.max(all_pts, axis=0)
        xr, yr, zr = xmax - xmin, ymax - ymin, zmax - zmin
        r = 0.5 * max(xr, yr, zr, 1e-3)
        cx, cy, cz = (xmax + xmin)/2, (ymax + ymin)/2, (zmax + zmin)/2

        self.ax.set_xlim(cx - r, cx + r)
        self.ax.set_ylim(cy - r, cy + r)
        gz = self.ground_z.get()
        lo = min(gz - 0.1*r, cz - r)
        hi = max(cz + r, gz + 0.5*r)
        self.ax.set_zlim(lo, hi)

        # 地面を描き直し（範囲に合わせて）
        for coll in list(self.ax.collections):
            if getattr(coll, "_is_ground", False):
                coll.remove()
        X, Y = np.meshgrid(np.linspace(cx - r, cx + r, 2), np.linspace(cy - r, cy + r, 2))
        Z = np.full_like(X, gz)
        ground = self.ax.plot_surface(X, Y, Z, alpha=0.08, edgecolor='none')
        setattr(ground, "_is_ground", True)

    # ---------- フィット ----------
    def on_fit(self):
        vis = self._gather_visible_points()  # 可視○×を集める既存関数そのまま流用
        self.txt_info.delete("1.0", tk.END)
        any_fit = False
        any_fit |= self._fit_draw_one("SIM",  vis["sim"]["contact"],  vis["sim"]["noncontact"],  "green")
        any_fit |= self._fit_draw_one("REAL", vis["real"]["contact"], vis["real"]["noncontact"], "yellow")
        if not any_fit:
            messagebox.showinfo("情報", "フィット可能な○点がありません．")
        self.canvas.draw_idle()

    def _fit_draw_one(self, kind: str, P_contact: np.ndarray, P_empty: np.ndarray, surf_color: str) -> bool:
        pts=[]
        if P_contact.size:
            for x,y,z in P_contact: pts.append((float(x),float(y),float(z),True))
        if P_empty.size:
            for x,y,z in P_empty:  pts.append((float(x),float(y),float(z),False))

        fit = fit_ellipsoid_with_empty_and_ground(
            pts,
            z_ground=self.ground_z.get(),
            w_sphere=0.5, ratio_cap=1.7, alpha_align_z=0.5,
            EPS_CONTACT=1e-6, xy_search_rad=10.0, xy_steps=5
        )
        if not fit: return False

        c,R,axes = fit["center"], fit["R"], fit["axes"]
        X,Y,Z = make_ellipsoid_mesh(c, axes, R, nu=48, nv=24)
        self.ax.plot_surface(X,Y,Z, alpha=0.25, linewidth=0, shade=True, color=surf_color)

        rz = R[2,:]
        Lz = float(np.sqrt((axes[0]*rz[0])**2 + (axes[1]*rz[1])**2 + (axes[2]*rz[2])**2))
        min_z = c[2]-Lz
        roll,pitch,yaw = rot_to_rpy_ZYX(R)
        self.txt_info.insert(tk.END,
            f"[{kind}] center=({c[0]:.2f},{c[1]:.2f},{c[2]:.2f})  "
            f"axes=({axes[0]:.2f},{axes[1]:.2f},{axes[2]:.2f})  "
            f"RPY=({roll:.1f},{pitch:.1f},{yaw:.1f})\n"
            f"      ground_z={self.ground_z.get():.2f}, min_z={min_z:.2f}  "
            f"constraints: contact_ok=True, empty_metric={fit['empty_metric']:.3f}\n"
        )
        return True

# =========================
# 起動
# =========================
if __name__ == "__main__":
    app = EllipsoidGUI()
    # 既知ファイルがあれば自動セット
    default_sim = "/mnt/data/sim_result_20251103_162020.json"
    default_real = "/mnt/data/log_real.json"
    if os.path.isfile(default_sim): app.ent_sim.insert(0, default_sim)
    if os.path.isfile(default_real): app.ent_real.insert(0, default_real)
    app.mainloop()