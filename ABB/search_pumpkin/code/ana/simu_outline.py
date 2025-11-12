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
def rot_to_rpy_ZYX(R: np.ndarray) -> Tuple[float, float, float]:
    """回転行列 → RPY(ZYX: yaw-pitch-roll) [deg]"""
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


def pca_fit_from_contacts(points_xyz: np.ndarray) -> Optional[Dict[str, np.ndarray]]:
    """
    ○点のみから PCA で楕円体の主軸・中心・軸長スケール基準を得る（等方一括係数前）．
    返すもの: dict(center, R(3x3), lam(3,))
      - center: 平均
      - R: 固有ベクトル（列が主軸）
      - lam: 共分散の固有値（降順）
    """
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


def make_ellipsoid_mesh(center: np.ndarray, axes: np.ndarray, R: np.ndarray, nu: int = 48, nv: int = 24):
    u = np.linspace(0, 2 * np.pi, nu)
    v = np.linspace(0, np.pi, nv)
    uu, vv = np.meshgrid(u, v)
    xs = np.cos(uu) * np.sin(vv)
    ys = np.sin(uu) * np.sin(vv)
    zs = np.cos(vv)
    S = np.stack([xs, ys, zs], axis=-1)
    S = S * axes
    S = S @ R.T + center
    return S[..., 0], S[..., 1], S[..., 2]


# 実機ログ 3x3 ピン配置（行=Y，列=X）
PIN_OFFSETS_RC = {
    1: (-1, -1), 2: (0, -1), 3: (1, -1),
    4: (-1,  0), 5: (0,  0), 6: (1,  0),
    7: (-1,  1), 8: (0,  1), 9: (1,  1),
}


# =========================
# 新フィッティング本体
# =========================
def _solve_scale_with_ground_and_empty_rays(
    P_contact: np.ndarray,
    P_empty: np.ndarray,
    c_init: np.ndarray, R: np.ndarray, lam: np.ndarray,
    z_ground: float, quantile: float = 0.80,
    t_lo: float = 1e-3, t_hi: float = 1e3, iters: int = 48,
    delta_empty: float = 0.0,  # 追加: 空振りマージン
):
    """
    接地条件(min_z=z_ground)を満たした状態で,
    - ○: quantile(d^2) <= 1   （表面の一部にロバストに合う）
    - ×: 各点 p=(x,y,z0) の「上向き縦レイ」 r(s)=p+s*[0,0,1], s>=0 が楕円体と交差しない
         （= そのレイに沿う d^2(s) の最小値 > 1 + delta_empty）
    の両方を満たす最大の t を二分探索で求める．
    戻り: (center, axes, t, feasible:bool, contact_ok:bool, empty_ok:bool)
    """
    lam = np.asarray(lam, float)
    rz = R[2, :]  # 各主軸の z 成分

    def axes_of(t: float) -> np.ndarray:
        return np.sqrt(np.maximum(1e-12, (t*t) * lam))

    def center_of(t: float) -> np.ndarray:
        a = axes_of(t)
        Lz_sq = (a[0]*rz[0])**2 + (a[1]*rz[1])**2 + (a[2]*rz[2])**2
        Lz = math.sqrt(max(1e-12, Lz_sq))
        c = c_init.copy()
        c[2] = z_ground + Lz  # 厳密接地
        return c, a

    ez = np.array([0.0, 0.0, 1.0], float)

    def quantile_d2_contacts(t: float) -> float:
        c, a = center_of(t)
        if not P_contact.size:
            return 0.0
        Q = (P_contact - c) @ R
        inv2 = 1.0 / np.maximum(1e-12, a*a)
        d2 = np.sum(Q * (Q * inv2), axis=1)
        return float(np.quantile(d2, quantile))

    def min_d2_empty_rays(t: float) -> float:
        # 各×点に対し, 縦レイ r(s)=p+s*ez の d^2(s) を最小化（s>=0）
        if not P_empty.size:
            return float("+inf")
        c, a = center_of(t)
        A = R @ np.diag(1.0/np.maximum(1e-12, a*a)) @ R.T  # A = R diag(1/a^2) R^T
        alpha = float(ez.T @ A @ ez)                        # s^2 係数
        vals = []
        for p in P_empty:
            u = p - c
            beta = float(u.T @ A @ ez)                      # s 係数の半分（実際は 2*beta）
            gamma = float(u.T @ A @ u)
            # f(s) = alpha s^2 + 2 beta s + gamma, s>=0
            if alpha <= 1e-18:
                fmin = gamma
            else:
                s_star = max(0.0, -beta / alpha)
                fmin = alpha*s_star*s_star + 2.0*beta*s_star + gamma
            vals.append(fmin)
        return float(np.min(vals))

    # contact側の下限 t_contact_min
    lo, hi = t_lo, t_hi
    for _ in range(iters):
        mid = math.sqrt(lo*hi)
        q = quantile_d2_contacts(mid)
        if q > 1.0:
            lo = mid
        else:
            hi = mid
    t_contact_min = math.sqrt(lo*hi)

    # empty側の上限 t_empty_max（> 1 + delta_empty を保てる最大の t）
    lo, hi = t_lo, t_hi
    for _ in range(iters):
        mid = math.sqrt(lo*hi)
        m = min_d2_empty_rays(mid)
        if m > 1.0 + delta_empty:
            lo = mid
        else:
            hi = mid
    t_empty_max = math.sqrt(lo*hi)

    feasible = t_empty_max >= t_contact_min
    if feasible:
        t_star = t_contact_min      # 両立域の下端＝最小サイズ
    else:
        t_star = t_empty_max        # 両立不可→×優先

    c_star, a_star = center_of(t_star)
    q_star = quantile_d2_contacts(t_star)
    m_star = min_d2_empty_rays(t_star)
    return c_star, a_star, t_star, feasible, (q_star <= 1.0+1e-6), (m_star > 1.0 + delta_empty - 1e-6)


def fit_ellipsoid_with_empty_and_ground(
    points_with_flags,
    z_ground: float = 0.0,
    quantile: float = 0.80,
    w_sphere: float = 0.3,         # 追加: 球面化バイアス（0で無効）
    ratio_cap: float = 1.8,        # 追加: 軸比上限（1.0→完全球）
    alpha_align_z: float = 0.5,    # 追加: Z軸整列の強さ（0で無効）
    delta_empty: float = 0.10,     # 追加: 空振りのマージン
):
    """
    points_with_flags: [(x,y,z, contact_bool), ...]
      - contact_bool=True  → 接触点（表面上 ≈ F=0）
      - contact_bool=False → 同一(x,y)・そのzから +Z 方向は“空”（縦レイ非交差）
    """
    if not points_with_flags:
        return None

    P = np.asarray([(x, y, z) for (x, y, z, _) in points_with_flags], float)
    Cmask = np.asarray([bool(c) for (_, _, _, c) in points_with_flags], bool)

    # ○命中点／×非命中点の分離
    P_contact = P[Cmask]
    P_empty   = P[~Cmask]

    # 姿勢推定の安定性のため最低4点の接触点を要求
    if P_contact.shape[0] < 4:
        return None

    # 1) PCAで初期中心・姿勢・固有値（スケール基準）
    base = pca_fit_from_contacts(P_contact)
    if base is None:
        return None
    c0, R, lam = base["center"], base["R"], base["lam"]

    # (C) Z軸整列の弱拘束
    if alpha_align_z > 0.0:
        ez = np.array([0.0, 0.0, 1.0])
        v3 = R[:, 2]
        v3b = (1 - alpha_align_z) * v3 + alpha_align_z * ez
        v3b /= max(1e-12, np.linalg.norm(v3b))
        # v1 は既存軸をベースに，v2 を外積で作り直す
        v1 = R[:, 0]; v1 /= max(1e-12, np.linalg.norm(v1))
        v2 = np.cross(v3b, v1);  n2 = np.linalg.norm(v2)
        if n2 < 1e-9:
            v1 = np.array([1,0,0], float); v2 = np.cross(v3b, v1); v2/=np.linalg.norm(v2)
        else:
            v2 /= n2
        v1 = np.cross(v2, v3b)
        R = np.stack([v1, v2, v3b], axis=1)

    # (A) 球面化バイアス：固有値を平均へ寄せる
    if w_sphere > 0.0:
        m = float(np.mean(lam))
        lam = (1 - w_sphere) * lam + w_sphere * m

    # (B) 軸比キャップ（lam は軸長^2 に比例）
    if ratio_cap is not None and ratio_cap >= 1.0:
        # 最大/最小の比が ratio_cap^2 を超えないよう補正
        i_max = int(np.argmax(lam)); i_min = int(np.argmin(lam))
        if lam[i_min] < 1e-12:
            lam[i_min] = 1e-12
        ratio = math.sqrt(lam[i_max] / lam[i_min])
        if ratio > ratio_cap:
            target_max = (ratio_cap ** 2) * lam[i_min]
            if lam[i_max] > target_max:
                lam[i_max] = target_max

    # 2) 接地＋縦レイ非交差＋接触点分位適合 を同時に満たすスケール t を探索（delta_empty 反映）
    c, axes, t, feasible, contact_ok, empty_ok = _solve_scale_with_ground_and_empty_rays(
        P_contact=P_contact,
        P_empty=P_empty,
        c_init=c0, R=R, lam=lam,
        z_ground=z_ground, quantile=quantile,
        delta_empty=delta_empty,
    )

    return dict(center=c, R=R, axes=axes, t=t,
                feasible=bool(feasible), contact_ok=bool(contact_ok), empty_ok=bool(empty_ok))

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
        vis = self._gather_visible_points()

        self.txt_info.delete("1.0", tk.END)
        any_fit = False

        # SIM
        sim_ok = self._fit_draw_one(kind="SIM",
                                    P_contact=vis["sim"]["contact"],
                                    P_empty=vis["sim"]["noncontact"],
                                    surf_color="green")
        any_fit = any_fit or sim_ok

        # REAL
        real_ok = self._fit_draw_one(kind="REAL",
                                    P_contact=vis["real"]["contact"],
                                    P_empty=vis["real"]["noncontact"],
                                    surf_color="yellow")
        any_fit = any_fit or real_ok

        if not any_fit:
            messagebox.showinfo("情報", "フィット可能な○点がありません（表示ONかつ選択IDに該当する○点のみを使用）．")

        self.canvas.draw_idle()
    
    


    def _fit_draw_one(self, kind: str, P_contact: np.ndarray, P_empty: np.ndarray, surf_color: str) -> bool:
        # 入力点を (x,y,z,contact_bool) 形式へまとめる
        pts = []
        if P_contact.size:
            for x, y, z in P_contact:
                pts.append((float(x), float(y), float(z), True))
        if P_empty.size:
            for x, y, z in P_empty:
                pts.append((float(x), float(y), float(z), False))

        # 横広がり抑制向けの推奨プリセット（必要に応じて数値だけ変更）
        W_SPHERE    = 0.5   # 0.0〜0.7（大きいほど球に近づく）
        RATIO_CAP   = 1.7   # 1.3〜2.0（最大/最小軸比の上限）
        ALIGN_Z     = 0.5   # 0.0〜0.8（主軸をZに寄せる）
        DELTA_EMPTY = 0.12  # 0.00〜0.20（空振りマージン）

        fit = fit_ellipsoid_with_empty_and_ground(
            points_with_flags=pts,
            z_ground=self.ground_z.get(),
            quantile=0.80,
            w_sphere=W_SPHERE,
            ratio_cap=RATIO_CAP,
            alpha_align_z=ALIGN_Z,
            delta_empty=DELTA_EMPTY,
        )
        if not fit:
            return False

        c, R, axes = fit["center"], fit["R"], fit["axes"]
        X, Y, Z = make_ellipsoid_mesh(c, axes, R, nu=48, nv=24)
        self.ax.plot_surface(X, Y, Z, alpha=0.25, linewidth=0, shade=True, color=surf_color)

        # 情報テキスト
        rz = R[2, :]
        Lz = math.sqrt((axes[0]*rz[0])**2 + (axes[1]*rz[1])**2 + (axes[2]*rz[2])**2)
        min_z = c[2] - Lz
        roll, pitch, yaw = rot_to_rpy_ZYX(R)
        lines = []
        lines.append(f"[{kind}] center=({c[0]:.2f},{c[1]:.2f},{c[2]:.2f})")
        lines.append(f"      axes(a,b,c)=({axes[0]:.2f},{axes[1]:.2f},{axes[2]:.2f})  RPY(deg)=({roll:.1f},{pitch:.1f},{yaw:.1f})")
        lines.append(f"      ground_z={self.ground_z.get():.2f}, min_z={min_z:.2f}  (ground contact enforced)")
        lines.append(f"      params: w_sphere={W_SPHERE}, ratio_cap={RATIO_CAP}, alignZ={ALIGN_Z}, d_empty={DELTA_EMPTY}")
        lines.append(f"      constraints: contact_ok={fit['contact_ok']}, empty_ok={fit['empty_ok']}, feasible_both={fit['feasible']}")
        if not fit["feasible"]:
            lines.append("  ⚠ 注意: ×(縦レイ非交差)と○(分位点適合)を同時に満たすtが存在しません．×を優先した最小化で推定しています．")
        self.txt_info.insert(tk.END, "\n".join(lines) + "\n")
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