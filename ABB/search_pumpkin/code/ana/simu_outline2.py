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

# STL描画（Matplotlib）用
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

# STL読み込み（PyVista）
try:
    import pyvista as pv
except Exception as e:
    pv = None  # 未インストール時はGUIで警告する


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


def _pca_basis(points_xyz: np.ndarray):
    """PCAで中心 c0，回転 R（列=主軸），固有値 lam（降順）を返す"""
    P = np.asarray(points_xyz, float)
    c0 = P.mean(axis=0)
    Q  = P - c0
    C  = np.cov(Q.T) + 1e-12*np.eye(3)
    lam, V = np.linalg.eigh(C)
    idx = np.argsort(lam)[::-1]
    lam = lam[idx]; R = V[:, idx]
    return c0, R, lam

def _regularize_axes_from_lam(lam, w_sphere=0.5, ratio_cap=1.7):
    """
    固有値 lam から軸長の基準 s= sqrt(lam_reg) を作る
    - w_sphere: 球面化（0..1）
    - ratio_cap: 軸比上限（max/min <= ratio_cap）
    """
    lam = np.maximum(1e-12, np.asarray(lam, float))
    if w_sphere > 0:
        m = float(np.mean(lam))
        lam = (1.0 - w_sphere)*lam + w_sphere*m
    s = np.sqrt(lam)  # 未スケールの基準
    # ratio cap（最小軸を基準にクリップ）
    amin = float(np.min(s))
    s = np.minimum(s, amin*ratio_cap)
    return s  # (= 未スケールの a0,b0,c0)

def _A_from_R_axes(R, axes):
    """A = R diag(1/a^2,1/b^2,1/c^2) R^T"""
    inv2 = 1.0/np.maximum(1e-12, np.asarray(axes, float)**2)
    return R @ np.diag(inv2) @ R.T

def _Ainv_from_R_axes(R, axes):
    """A^{-1} = R diag(a^2,b^2,c^2) R^T"""
    return R @ np.diag(np.asarray(axes, float)**2) @ R.T

def _d2_with(R, axes, c, P):
    """Mahalanobis d^2（境界=1）"""
    A = _A_from_R_axes(R, axes)
    X = np.asarray(P, float) - c
    return np.sum((X @ A) * X, axis=1)

def _contact_quantile_t(P_contact, c0, R, s0, quantile=0.80):
    """
    未スケール軸 s0 に対して ○点の d^2 を計算し，
    q分位が1となるよう t を決める（d'^2 = d^2 / t^2）
    """
    d2 = _d2_with(R, s0, c0, P_contact)  # ここは s0 のまま（t=1）
    if d2.size == 0:
        return 1.0
    q = float(np.quantile(d2, quantile))
    return math.sqrt(max(q, 1e-12))

def _empty_upper_t(P_empty, c0, R, s0):
    """
    ×（空振り）縦レイ非交差を満たす t の上限を計算．
    クアドラティック (r + s e_z)^T A' (r + s e_z) = 1 の +Z 交点 s を解析，
    s_max <= 0 となる条件から，B>0 の場合 t^2 <= C に簡約（A0,B,Cは t 依存を除去）。
      A0 = e_z^T A0 e_z
      B  = 2 r^T A0 e_z
      C  = r^T A0 r
    ここで A' = (1/t^2) A0，r = (x,y,z) - c0，e_z = [0,0,1]^T
    """
    if P_empty.size == 0:
        return np.inf
    A0 = _A_from_R_axes(R, s0)   # 未スケール
    ez = np.array([0.,0.,1.])
    A0_ez = A0 @ ez
    t2_ubs = []
    for p in np.asarray(P_empty, float):
        r = p - c0
        B = 2.0 * (r @ A0_ez)
        if B <= 0.0:
            # +Z方向の交差を“持たない／遠ざかる”ので制約なし
            continue
        C = float(r @ (A0 @ r))
        t2_ubs.append(max(C, 0.0))  # t^2 <= C
    if not t2_ubs:
        return np.inf
    return math.sqrt(max(1e-12, min(t2_ubs)))

def _ground_contact_center(cz_ref, R, axes, z_ground):
    """
    厳密接地：min_z = z_ground を満たす c_z を閉形式で計算
    Lz = sqrt(e_z^T A^{-1} e_z) なので c_z = z_ground + Lz
    """
    Ainv = _Ainv_from_R_axes(R, axes)
    ez = np.array([0.,0.,1.])
    Lz = math.sqrt(float(ez @ (Ainv @ ez)))
    return z_ground + Lz

def fit_ellipsoid_with_empty_and_ground(
    points_with_flags, z_ground=0.0, quantile=0.80,
    w_sphere=0.5, ratio_cap=1.7, alpha_align_z=0.5, delta_empty=0.12
):
    """
    入力: points_with_flags = [(x,y,z, is_contact_bool), ...]
    出力: dict(center=c, R=R, axes=[a,b,c], feasible, contact_ok, empty_ok) or None
    ・○は q 分位までを内包（d^2 / t^2 <= 1）
    ・×は +Z 縦レイに“交差しない”（解析条件 → t <= sqrt(C) for B>0）
    ・接地：min_z = z_ground を厳密保証
    """
    P = np.asarray([(x,y,z) for (x,y,z,_) in points_with_flags], float)
    F = np.asarray([bool(b) for (_,_,_,b) in points_with_flags], bool)
    P_contact = P[F]; P_empty = P[~F]

    if P_contact.shape[0] < 4:
        return None

    # 1) PCA 基準
    c0, R, lam = _pca_basis(P_contact)

    # 2) 軸の正則化（球面化＋軸比制限）
    s0 = _regularize_axes_from_lam(lam, w_sphere=w_sphere, ratio_cap=ratio_cap)

    # 3) スケール t を決定：○分位 と ×上限 を同時に満たす最大 t
    t_contact = _contact_quantile_t(P_contact, c0, R, s0, quantile=quantile)
    t_empty_ub = _empty_upper_t(P_empty, c0, R, s0)
    # マージン（空レイにゆとりを持たせる）
    if np.isfinite(t_empty_ub):
        t_empty_ub = max(0.0, t_empty_ub - delta_empty)
    t = min(t_contact, t_empty_ub) if np.isfinite(t_empty_ub) else t_contact

    feasible = (t_contact <= (t_empty_ub + 1e-9)) if np.isfinite(t_empty_ub) else True

    # 4) 最終軸
    axes = t * s0

    # 5) 接地で c_z を確定（c_x,c_y はPCA中心のまま）
    c = c0.copy()
    c[2] = _ground_contact_center(c[2], R, axes, z_ground)

    # 6) 妥当性の記録
    d2c = _d2_with(R, axes, c, P_contact)
    contact_ok = bool(np.quantile(d2c, quantile) <= 1.0 + 1e-6)

    empty_ok = True
    if P_empty.size:
        A = _A_from_R_axes(R, axes)
        ez = np.array([0.,0.,1.])
        Aez = A @ ez
        for p in P_empty:
            r = p - c
            # s roots independent of t 形式（Aは最終）
            a = float(ez @ Aez)
            b = 2.0 * float(r @ Aez)
            ccoef = float(r @ (A @ r) - 1.0)
            D = b*b - 4*a*ccoef
            if D >= 0:
                s1 = (-b - math.sqrt(D)) / (2*a)
                s2 = (-b + math.sqrt(D)) / (2*a)
                if max(s1, s2) > 0.0:  # +Z 側に交点あり → 失敗
                    empty_ok = False
                    break

    return dict(center=c, R=R, axes=axes, feasible=feasible, contact_ok=contact_ok, empty_ok=empty_ok)



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

        # ---- 表示フラグ（点群表示）----
        self.show_sim  = tk.BooleanVar(value=True)   # シミュ点群（青）
        self.show_real = tk.BooleanVar(value=True)   # 実機点群（赤）

        # ---- 楕円体の表示フラグ ----
        self.show_sim_fit  = tk.BooleanVar(value=True)   # 楕円体(シミュ)
        self.show_real_fit = tk.BooleanVar(value=True)   # 楕円体(実機)

        # ---- 地面高さ ----
        self.ground_z = tk.DoubleVar(value=0.0)

        # ---- 実機再構成パラメータ ----
        self.dx = tk.DoubleVar(value=30.0)
        self.dy = tk.DoubleVar(value=30.0)
        self.z0 = tk.DoubleVar(value=80.0)
        self.k_scale = tk.DoubleVar(value=1.0)

        # ---- ID選択状態 ----
        self.selected_ids: Set[str] = set()

        # ---- STL 関連 ----
        self.stl_path = tk.StringVar(value="")
        self.show_stl_mesh = tk.BooleanVar(value=True)   # STLメッシュ表示
        self.show_stl_fit  = tk.BooleanVar(value=True)   # 楕円体(STL)

        self._stl_mesh = None              # PyVistaメッシュ
        self._stl_points_np = None         # np.ndarray(N,3)
        self._stl_artist = None            # STLのPoly3DCollection

        # ---- 楕円体の直近結果（再描画用）----
        self._sim_fit_last  = None   # {"center":c,"R":R,"axes":axes}
        self._real_fit_last = None
        self._stl_fit_last  = None

        # ---- 楕円体の描画ハンドル（再描画時に消すため）----
        self._sim_fit_artist  = None
        self._real_fit_artist = None
        self._stl_fit_artist  = None

        self._build_ui()
        self._init_plot()

    # ---------- UI ----------
    def _build_ui(self):
        self.columnconfigure(0, weight=1)
        self.columnconfigure(1, weight=0)
        self.rowconfigure(0, weight=1)

        # 左：3D＋ツールバー
        left = ttk.Frame(self); left.grid(row=0, column=0, sticky="nsew", padx=6, pady=6)
        left.rowconfigure(0, weight=1); left.rowconfigure(1, weight=0); left.columnconfigure(0, weight=1)

        self.fig = plt.figure(figsize=(9, 6.6))
        self.ax  = self.fig.add_subplot(111, projection="3d")
        self.canvas = FigureCanvasTkAgg(self.fig, master=left)
        self.canvas.get_tk_widget().grid(row=0, column=0, sticky="nsew")
        tb = ttk.Frame(left); tb.grid(row=1, column=0, sticky="ew")
        self.toolbar = NavigationToolbar2Tk(self.canvas, tb); self.toolbar.update()

        # 右：操作
        right = ttk.Frame(self); right.grid(row=0, column=1, sticky="ns", padx=6, pady=6)
        for r in range(14): right.rowconfigure(r, weight=0)
        right.rowconfigure(13, weight=1); right.columnconfigure(0, weight=1)

        # ==== ファイル選択（3行：SIM/REAL/STL それぞれ 参照＋読込/描画）====
        frm_file = ttk.LabelFrame(right, text="ファイル選択"); frm_file.grid(row=0, column=0, sticky="ew", pady=4)
        frm_file.columnconfigure(1, weight=1)

        # シミュ
        ttk.Label(frm_file, text="シミュJSON:").grid(row=0, column=0, sticky="w")
        self.ent_sim = ttk.Entry(frm_file, width=38); self.ent_sim.grid(row=0, column=1, sticky="ew", padx=4)
        ttk.Button(frm_file, text="参照", command=self.on_browse_sim).grid(row=0, column=2)
        ttk.Button(frm_file, text="読み込み/描画", command=self.on_load_sim).grid(row=0, column=3, padx=2)

        # 実機
        ttk.Label(frm_file, text="実機JSON:").grid(row=1, column=0, sticky="w")
        self.ent_real = ttk.Entry(frm_file, width=38); self.ent_real.grid(row=1, column=1, sticky="ew", padx=4)
        ttk.Button(frm_file, text="参照", command=self.on_browse_real).grid(row=1, column=2)
        ttk.Button(frm_file, text="読み込み/描画", command=self.on_load_real).grid(row=1, column=3, padx=2)

        # STL
        ttk.Label(frm_file, text="STL:").grid(row=2, column=0, sticky="w")
        self.ent_stl = ttk.Entry(frm_file, textvariable=self.stl_path, width=38); self.ent_stl.grid(row=2, column=1, sticky="ew", padx=4)
        ttk.Button(frm_file, text="参照", command=self.on_browse_stl).grid(row=2, column=2)
        ttk.Button(frm_file, text="読み込み/描画", command=self.on_load_stl).grid(row=2, column=3, padx=2)

        # ==== 表示切替（点群＋STL＋各楕円体）====
        frm_view = ttk.LabelFrame(right, text="表示切替"); frm_view.grid(row=1, column=0, sticky="ew", pady=4)
        ttk.Checkbutton(frm_view, text="シミュ点群（青）", variable=self.show_sim,  command=self.redraw).grid(row=0, column=0, sticky="w")
        ttk.Checkbutton(frm_view, text="実機点群（赤）", variable=self.show_real, command=self.redraw).grid(row=1, column=0, sticky="w")
        ttk.Checkbutton(frm_view, text="STLメッシュ",     variable=self.show_stl_mesh, command=self.redraw).grid(row=2, column=0, sticky="w")
        ttk.Separator(frm_view, orient="horizontal").grid(row=3, column=0, sticky="ew", pady=3)
        ttk.Checkbutton(frm_view, text="楕円体（シミュ：緑）",   variable=self.show_sim_fit,  command=self.redraw).grid(row=4, column=0, sticky="w")
        ttk.Checkbutton(frm_view, text="楕円体（実機：黄）",     variable=self.show_real_fit, command=self.redraw).grid(row=5, column=0, sticky="w")
        ttk.Checkbutton(frm_view, text="楕円体（STL：橙）",      variable=self.show_stl_fit,  command=self.redraw).grid(row=6, column=0, sticky="w")

        # 自動ズーム/軸調整
        self.auto_view = tk.BooleanVar(value=True)
        ttk.Checkbutton(frm_view, text="自動ズーム/軸調整を有効化", variable=self.auto_view, command=self.redraw)\
            .grid(row=7, column=0, sticky="w")

        # ==== 地面制約 ====
        frm_ground = ttk.LabelFrame(right, text="地面制約"); frm_ground.grid(row=2, column=0, sticky="ew", pady=4)
        ttk.Label(frm_ground, text="地面高さ z =").grid(row=0, column=0, sticky="e")
        ttk.Entry(frm_ground, textvariable=self.ground_z, width=9).grid(row=0, column=1, sticky="w")

        # ==== 実機再構成 ====
        frm_real = ttk.LabelFrame(right, text="実機再構成設定"); frm_real.grid(row=3, column=0, sticky="ew", pady=4)
        for i in range(4): frm_real.columnconfigure(i, weight=1)
        ttk.Label(frm_real, text="dx[mm]").grid(row=0, column=0, sticky="e"); ttk.Entry(frm_real, textvariable=self.dx, width=8).grid(row=0, column=1, sticky="w")
        ttk.Label(frm_real, text="dy[mm]").grid(row=0, column=2, sticky="e"); ttk.Entry(frm_real, textvariable=self.dy, width=8).grid(row=0, column=3, sticky="w")
        ttk.Label(frm_real, text="Z0[mm]").grid(row=1, column=0, sticky="e"); ttk.Entry(frm_real, textvariable=self.z0, width=8).grid(row=1, column=1, sticky="w")
        ttk.Label(frm_real, text="k(押込→Z)").grid(row=1, column=2, sticky="e"); ttk.Entry(frm_real, textvariable=self.k_scale, width=8).grid(row=1, column=3, sticky="w")

        # ==== ID一覧 ====
        frm_ids = ttk.LabelFrame(right, text="ID 選択（クリックでトグル）"); frm_ids.grid(row=4, column=0, sticky="nsew", pady=4)
        frm_ids.rowconfigure(0, weight=1); frm_ids.columnconfigure(0, weight=1)
        self.lst_ids = tk.Listbox(frm_ids, selectmode=tk.SINGLE, height=12)
        self.lst_ids.grid(row=0, column=0, sticky="nsew")
        self.lst_ids.bind("<<ListboxSelect>>", self.on_id_click)
        sb = ttk.Scrollbar(frm_ids, orient="vertical", command=self.lst_ids.yview); sb.grid(row=0, column=1, sticky="ns")
        self.lst_ids.configure(yscrollcommand=sb.set)
        ttk.Button(frm_ids, text="クリア（全解除）", command=self.on_clear_selection).grid(row=1, column=0, columnspan=2, sticky="ew", pady=4)

        # ==== 楕円体フィッティング（SIM/REAL/STL の3ボタン）====
        frm_fit = ttk.LabelFrame(right, text="楕円体フィッティング"); frm_fit.grid(row=5, column=0, sticky="ew", pady=4)
        ttk.Button(frm_fit, text="シミュからフィット（○＋×）", command=self.on_fit_sim).grid(row=0, column=0, sticky="ew")
        ttk.Button(frm_fit, text="実機からフィット（○＋×）",   command=self.on_fit_real).grid(row=1, column=0, sticky="ew")
        ttk.Button(frm_fit, text="STLからフィット（○のみ）",   command=self.on_fit_stl).grid(row=2, column=0, sticky="ew")

        # ==== 情報欄 ====
        frm_info = ttk.LabelFrame(right, text="楕円体情報 / 整合性チェック"); frm_info.grid(row=6, column=0, sticky="nsew", pady=4)
        frm_info.rowconfigure(0, weight=1); frm_info.columnconfigure(0, weight=1)
        self.txt_info = tk.Text(frm_info, height=14, wrap="word"); self.txt_info.grid(row=0, column=0, sticky="nsew")
        sb2 = ttk.Scrollbar(frm_info, orient="vertical", command=self.txt_info.yview); sb2.grid(row=0, column=1, sticky="ns")
        self.txt_info.configure(yscrollcommand=sb2.set)

    def on_load_sim(self):
        sim_p = self.ent_sim.get().strip()
        if not sim_p:
            messagebox.showinfo("情報", "シミュJSONを指定してください．"); return
        if not os.path.isfile(sim_p) or not is_sim_file(sim_p):
            messagebox.showwarning("警告", "シミュJSONの形式が想定と異なります．"); return
        self.data.load_sim(sim_p)
        self._refresh_id_list()
        self.redraw()

    def on_load_real(self):
        real_p = self.ent_real.get().strip()
        if not real_p:
            messagebox.showinfo("情報", "実機JSONを指定してください．"); return
        if not os.path.isfile(real_p) or not is_real_log_file(real_p):
            messagebox.showwarning("警告", "実機JSONの形式が想定と異なります．"); return
        self.data.load_real(real_p, self.dx.get(), self.dy.get(), self.z0.get(), self.k_scale.get())
        self._refresh_id_list()
        self.redraw()

    def on_load_stl(self):
        path = self.stl_path.get().strip()
        if not path or not os.path.isfile(path):
            messagebox.showwarning("警告", "STLファイルが見つかりません．"); return
        if pv is None:
            messagebox.showwarning("警告", "PyVista が利用できません．'pip install pyvista' で導入してください．"); return
        try:
            mesh = pv.read(path).triangulate()
            self._stl_mesh = mesh
            self._stl_points_np = np.asarray(mesh.points, float)
            # STL再読込時は楕円体結果をクリア
            self._stl_fit_last = None
            self.redraw()
        except Exception as e:
            messagebox.showerror("エラー", f"STL読み込みに失敗しました:\n{e}")


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
        self._recolor_id_list()

    # ---------- STL: ファイル選択 ----------
    def on_browse_stl(self):
        p = filedialog.askopenfilename(title="STL ファイルを選択", filetypes=[("STL", "*.stl"), ("All", "*.*")])
        if p:
            self.stl_path.set(p)

    # ---------- STL: 読込/描画 ----------
    def on_load_stl(self):
        path = self.stl_path.get().strip()
        if not path or not os.path.isfile(path):
            messagebox.showwarning("警告", "STLファイルが見つかりません．"); return
        if pv is None:
            messagebox.showwarning("警告", "PyVista が利用できません．'pip install pyvista' で導入してください．"); return
        try:
            mesh = pv.read(path)
            # 三角形化（PyVistaは多くが三角面だが念のため）
            mesh = mesh.triangulate()
            self._stl_mesh = mesh
            self._stl_points_np = np.asarray(mesh.points, float) if (mesh is not None) else None
            # STLを読み直したら既存のSTLフィット結果はクリア
            self._stl_fit_last = None
            self.redraw()
        except Exception as e:
            messagebox.showerror("エラー", f"STL読み込みに失敗しました:\n{e}")

    # ---------- STL: Matplotlibへメッシュ描画 ----------
    def _draw_stl_mesh(self):
        # 既存メッシュ描画の掃除
        if self._stl_artist is not None:
            try:
                self._stl_artist.remove()
            except Exception:
                pass
            self._stl_artist = None

        if (not self.show_stl_mesh.get()) or (self._stl_mesh is None):
            return

        # faces -> Poly3DCollection
        m = self._stl_mesh
        # PyVistaのfacesは [n, i0, i1, i2, n, ...] 形式（n=3固定を想定）
        faces = m.faces.reshape((-1, 4))[:, 1:4]  # (F,3)
        V = np.asarray(m.points, float)          # (N,3)

        tris = [V[idx] for idx in faces]         # リスト化（頂点3x3）
        coll = Poly3DCollection(tris, facecolor=(0.8, 0.8, 0.9, 0.35), edgecolor=(0.4, 0.4, 0.5, 0.25))
        coll.set_zsort('average')  # 3Dの半透明交差の見栄え向上
        self._stl_artist = self.ax.add_collection3d(coll)

    # ---------- STL: 楕円体フィット ----------
    def on_fit_stl(self):
        if self._stl_mesh is None:
            messagebox.showinfo("情報", "先にSTLを読み込んでください．"); return

        P = np.asarray(self._stl_mesh.points, float)
        if P.shape[0] < 4:
            messagebox.showinfo("情報", "STL点数が少なすぎます．"); return

        # 以前の結果があるなら一旦消す（redrawで描き直し可）
        if self._stl_fit_artist is not None:
            try: self._stl_fit_artist.remove()
            except Exception: pass
            self._stl_fit_artist = None
        self._stl_fit_last = None

        ok = self._fit_draw_one(kind="STL", P_contact=P, P_empty=np.empty((0,3), float), surf_color="orange")
        if not ok:
            messagebox.showinfo("情報", "STLからのフィットに失敗しました．"); return
        self.canvas.draw_idle()


    # ---------- 選択 ----------
    def on_id_click(self, _):
        sel = self.lst_ids.curselection()
        if not sel: return
        id_str = self.lst_ids.get(sel[0])
        if id_str in self.selected_ids: self.selected_ids.remove(id_str)
        else: self.selected_ids.add(id_str)
        self._recolor_id_list()
        self.redraw()

    def on_clear_selection(self):
        self.selected_ids.clear()
        self._recolor_id_list()
        self.redraw()

    def _recolor_id_list(self):
        """Listbox内の各IDの文字色を，選択状態に応じて更新する．"""
        try:
            n = self.lst_ids.size()
            for i in range(n):
                id_str = self.lst_ids.get(i)
                fg = "blue" if id_str in self.selected_ids else "black"
                # Tk 8.6以降: itemconfig(index, foreground=...) が利用可
                self.lst_ids.itemconfig(i, foreground=fg)
        except Exception:
            # 互換落ち環境でも落とさない
            pass

    def _get_view_state(self):
        """現在の視点・軸範囲を取得"""
        try:
            return dict(
                elev=float(self.ax.elev),
                azim=float(self.ax.azim),
                xlim=self.ax.get_xlim3d(),
                ylim=self.ax.get_ylim3d(),
                zlim=self.ax.get_zlim3d(),
            )
        except Exception:
            return None

    def _apply_view_state(self, st):
        """視点・軸範囲を復元"""
        try:
            if "xlim" in st: self.ax.set_xlim(st["xlim"])
            if "ylim" in st: self.ax.set_ylim(st["ylim"])
            if "zlim" in st: self.ax.set_zlim(st["zlim"])
            if "elev" in st and "azim" in st:
                self.ax.view_init(elev=st["elev"], azim=st["azim"])
        except Exception:
            pass



    # ---------- 可視点収集（○/×分離） ----------
    def _gather_visible_points(self) -> Dict[str, Dict[str, np.ndarray]]:
        """
        戻り:
        {
            "sim":  {"sel_c": Nx3, "sel_n": Mx3, "oth_c": Kx3, "oth_n": Lx3},
            "real": {"sel_c": ...,  "sel_n": ...,  "oth_c": ...,  "oth_n": ...}
        }
        - show_* がOFFの種別は空配列になる
        - ID選択が無い場合（self.selected_idsが空）は，全点を"sel_*"側に入れる（通常配色）
        """
        res = {
            "sim":  {"sel_c": [], "sel_n": [], "oth_c": [], "oth_n": []},
            "real": {"sel_c": [], "sel_n": [], "oth_c": [], "oth_n": []},
        }
        any_sel = bool(self.selected_ids)

        # --- SIM ---
        if self.show_sim.get():
            for id_str, pts in self.data.sim_points_by_id.items():
                is_sel = (id_str in self.selected_ids) or (not any_sel)
                for x, y, z, c in pts:
                    if is_sel:
                        (res["sim"]["sel_c"] if c else res["sim"]["sel_n"]).append([x, y, z])
                    else:
                        (res["sim"]["oth_c"] if c else res["sim"]["oth_n"]).append([x, y, z])

        # --- REAL ---
        if self.show_real.get():
            for id_str, pts in self.data.real_points_by_id.items():
                is_sel = (id_str in self.selected_ids) or (not any_sel)
                for x, y, z, c in pts:
                    if is_sel:
                        (res["real"]["sel_c"] if c else res["real"]["sel_n"]).append([x, y, z])
                    else:
                        (res["real"]["oth_c"] if c else res["real"]["oth_n"]).append([x, y, z])

        # ndarray 化
        for k1 in res:
            for k2 in res[k1]:
                arr = np.array(res[k1][k2], float) if res[k1][k2] else np.empty((0, 3), float)
                res[k1][k2] = arr
        return res


    # ---------- 描画 ----------
    def redraw(self):
        # 自動調整OFF時はビューを保持
        prev_view = None
        if hasattr(self, "auto_view") and not self.auto_view.get():
            prev_view = self._get_view_state()

        self._init_plot()
        vis = self._gather_visible_points()
        any_sel = bool(self.selected_ids)

        def scatter_points(arr: np.ndarray, marker: str, color: str, alpha: float):
            if arr.size:
                self.ax.scatter(arr[:, 0], arr[:, 1], arr[:, 2],
                                marker=marker, s=32, alpha=alpha, color=color)

        gray = "0.6"; alpha_gray = 0.35

        # ---- 点群描画（選択があると未選択はグレー半透明）----
        if self.show_sim.get():
            if any_sel:
                scatter_points(vis["sim"]["oth_c"], 'o', gray,  alpha_gray)
                scatter_points(vis["sim"]["oth_n"], 'x', gray,  alpha_gray)
            scatter_points(vis["sim"]["sel_c"],  'o', "blue", 0.95)
            scatter_points(vis["sim"]["sel_n"],  'x', "blue", 0.95)

        if self.show_real.get():
            if any_sel:
                scatter_points(vis["real"]["oth_c"], 'o', gray,  alpha_gray)
                scatter_points(vis["real"]["oth_n"], 'x', gray,  alpha_gray)
            scatter_points(vis["real"]["sel_c"], 'o', "red",  0.95)
            scatter_points(vis["real"]["sel_n"], 'x', "red",  0.95)

        # ---- STLメッシュ ----
        # 既存STLメッシュを消してから必要なら再描画
        if self._stl_artist is not None:
            try: self._stl_artist.remove()
            except Exception: pass
            self._stl_artist = None
        if self.show_stl_mesh.get() and (self._stl_mesh is not None):
            m = self._stl_mesh
            faces = m.faces.reshape((-1, 4))[:, 1:4]
            V = np.asarray(m.points, float)
            tris = [V[idx] for idx in faces]
            coll = Poly3DCollection(tris, facecolor=(0.8,0.8,0.9,0.35), edgecolor=(0.4,0.4,0.5,0.25))
            coll.set_zsort('average')
            self._stl_artist = self.ax.add_collection3d(coll)

        # ---- 楕円体（SIM/REAL/STL）再描画：フラグONかつ直近結果があるときだけ ----
        for kind, flg, last, holder_name, color in [
            ("SIM",  self.show_sim_fit,  self._sim_fit_last,  "_sim_fit_artist",  "green"),
            ("REAL", self.show_real_fit, self._real_fit_last, "_real_fit_artist", "yellow"),
            ("STL",  self.show_stl_fit,  self._stl_fit_last,  "_stl_fit_artist",  "orange"),
        ]:
            # 既存を消す
            artist = getattr(self, holder_name)
            if artist is not None:
                try: artist.remove()
                except Exception: pass
                setattr(self, holder_name, None)

            # 再描画
            if flg.get() and (last is not None):
                c, R, axes = last["center"], last["R"], last["axes"]
                X, Y, Z = make_ellipsoid_mesh(c, axes, R, nu=48, nv=24)
                artist = self.ax.plot_surface(X, Y, Z, alpha=0.25, linewidth=0, shade=True, color=color)
                setattr(self, holder_name, artist)

        # ---- 自動スケール ----
        if self.auto_view.get():
            def stack_all(d):
                return np.vstack([d["sel_c"], d["sel_n"], d["oth_c"], d["oth_n"]]) \
                    if any(v.size for v in d.values()) else np.empty((0, 3))
            sim_all  = stack_all(vis["sim"])  if self.show_sim.get()  else np.empty((0,3))
            real_all = stack_all(vis["real"]) if self.show_real.get() else np.empty((0,3))
            stl_pts  = self._stl_points_np if (self._stl_points_np is not None and self.show_stl_mesh.get()) else np.empty((0,3))
            all_for_auto = np.vstack([arr for arr in (sim_all, real_all, stl_pts) if arr.size])
            tmp = {"sim": {"contact": all_for_auto if all_for_auto.size else np.empty((0,3)),
                        "noncontact": np.empty((0,3))},
                "real":{"contact": np.empty((0,3)), "noncontact": np.empty((0,3))}}
            self._autoscale_equal(tmp)
        else:
            gz = self.ground_z.get()
            x0, x1 = self.ax.get_xlim3d(); y0, y1 = self.ax.get_ylim3d()
            X, Y = np.meshgrid([x0, x1], [y0, y1]); Z = np.full_like(X, gz)
            ground = self.ax.plot_surface(X, Y, Z, alpha=0.08, edgecolor='none'); setattr(ground, "_is_ground", True)

        if prev_view is not None:
            self._apply_view_state(prev_view)

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
        # 可視点を収集（_gather_visible_points は sel_c/sel_n/oth_c/oth_n を返す前提）
        vis = self._gather_visible_points()

        self.txt_info.delete("1.0", tk.END)
        any_fit = False

        def pick_colored(kind: str):
            """
            色が付いている点だけをフィット対象にする。
            - 選択あり: sel_*（選択ID）だけ使う
            - 選択なし: sel_* に全点が入っている設計なのでそのまま使う
            返り値: (P_contact, P_empty) いずれも (N,3) ndarray
            """
            blk = vis[kind]
            P_contact = blk["sel_c"]  # ○
            P_empty   = blk["sel_n"]  # ×（縦レイ否定制約として扱う）
            return P_contact, P_empty

        # ---- SIM（緑） ----
        P_contact_sim, P_empty_sim = pick_colored("sim")
        ok_sim = self._fit_draw_one(
            kind="SIM",
            P_contact=P_contact_sim,
            P_empty=P_empty_sim,
            surf_color="green"
        )
        any_fit = any_fit or ok_sim

        # ---- REAL（黄） ----
        P_contact_real, P_empty_real = pick_colored("real")
        ok_real = self._fit_draw_one(
            kind="REAL",
            P_contact=P_contact_real,
            P_empty=P_empty_real,
            surf_color="yellow"
        )
        any_fit = any_fit or ok_real

        if not any_fit:
            messagebox.showinfo("情報", "フィット可能な○点がありません（表示ONかつ“色が付いている”点のみを使用）．")

        self.canvas.draw_idle()

    def _fit_draw_one(self, kind: str, P_contact: np.ndarray, P_empty: np.ndarray, surf_color: str) -> bool:
        """
        kind: "SIM" | "REAL" | "STL"
        P_contact: Nx3（○）
        P_empty  : Mx3（×）
        """
        if P_contact.size < 4:
            return False

        # 1) points_with_flags へパック（GUIの○/×をそのまま反映）
        pts = []
        for x,y,z in np.asarray(P_contact, float): pts.append((float(x),float(y),float(z), True))
        for x,y,z in np.asarray(P_empty,   float): pts.append((float(x),float(y),float(z), False))

        # 2) アルゴリズム実行（パラメータはここで一元管理）
        fit = fit_ellipsoid_with_empty_and_ground(
            points_with_flags=pts,
            z_ground=self.ground_z.get(),
            quantile=0.80,          # ←必要に応じ GUI化可
            w_sphere=0.5,
            ratio_cap=1.7,
            alpha_align_z=0.5,      # ※現実装は回転行列Rを保持（Z整列は弱め）
            delta_empty=0.12,
        )
        if not fit:
            return False

        c, R, axes = fit["center"], fit["R"], fit["axes"]

        # 3) 既存の同種アーティストを消して，描画
        holder = {"SIM":"_sim_fit_artist", "REAL":"_real_fit_artist", "STL":"_stl_fit_artist"}[kind]
        old = getattr(self, holder, None)
        if old is not None:
            try: old.remove()
            except Exception: pass
            setattr(self, holder, None)

        X, Y, Z = make_ellipsoid_mesh(c, axes, R, nu=48, nv=24)
        surf = self.ax.plot_surface(X, Y, Z, alpha=0.25, linewidth=0, shade=True, color=surf_color)

        # 4) 結果保持（redrawで再描画）
        last_holder = {"SIM":"_sim_fit_last", "REAL":"_real_fit_last", "STL":"_stl_fit_last"}[kind]
        setattr(self, last_holder, {"center":c, "R":R, "axes":axes})
        setattr(self, holder, surf)

        # 5) 情報表示
        rz = R[2,:]
        Lz = math.sqrt((axes[0]*rz[0])**2 + (axes[1]*rz[1])**2 + (axes[2]*rz[2])**2)
        min_z = c[2] - Lz
        roll, pitch, yaw = rot_to_rpy_ZYX(R)
        lines = []
        lines.append(f"[{kind}] center=({c[0]:.2f},{c[1]:.2f},{c[2]:.2f})")
        lines.append(f"      axes(a,b,c)=({axes[0]:.2f},{axes[1]:.2f},{axes[2]:.2f})  RPY(deg)=({roll:.1f},{pitch:.1f},{yaw:.1f})")
        lines.append(f"      ground_z={self.ground_z.get():.2f}, min_z={min_z:.2f}  (ground contact)")
        lines.append(f"      constraints: contact_ok={fit['contact_ok']}, empty_ok={fit['empty_ok']}, feasible_both={fit['feasible']}")
        if not fit["feasible"]:
            lines.append("  ⚠ 注意: ×(縦レイ)と○(分位)の両立解が無く，上限側（×）を優先しました．")
        self.txt_info.insert(tk.END, "\n".join(lines) + "\n")
        return True

    def on_fit_sim(self):
        vis = self._gather_visible_points()
        blk = vis["sim"]
        P_contact, P_empty = blk["sel_c"], blk["sel_n"]  # 色が付いている=対象
        if not P_contact.size:
            messagebox.showinfo("情報", "シミュの○点がありません．"); return
        self._fit_draw_one("SIM", P_contact, P_empty, surf_color="green")
        self.canvas.draw_idle()

    def on_fit_real(self):
        vis = self._gather_visible_points()
        blk = vis["real"]
        P_contact, P_empty = blk["sel_c"], blk["sel_n"]
        if not P_contact.size:
            messagebox.showinfo("情報", "実機の○点がありません．"); return
        self._fit_draw_one("REAL", P_contact, P_empty, surf_color="yellow")
        self.canvas.draw_idle()

    def on_fit_stl(self):
        if self._stl_mesh is None:
            messagebox.showinfo("情報", "先にSTLを読み込んでください．"); return
        P = np.asarray(self._stl_mesh.points, float)
        if P.shape[0] < 4:
            messagebox.showinfo("情報", "STL点数が少なすぎます．"); return
        self._fit_draw_one("STL", P, np.empty((0,3), float), surf_color="orange")
        self.canvas.draw_idle()


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
