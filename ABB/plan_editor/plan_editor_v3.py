# -*- coding: utf-8 -*-
# plan_editor.py  (RPY指定 & 手首特異点回避バイアス対応版)
# 機能:
#  - pinart_info.json を読み込み
#  - 1回の押下で全ピン使用，範囲内格子(dx,dy)を優先して埋める（中心対称）
#  - 先端は重複ゼロを保証
#  - 表示: 2D/3D 切替（初期2D）
#  - 点B: 押下点（青）/ RPY は GUI 指定（＋微小バイアスを適用）
#  - 点A: 退避点（緑）= B をワールド+Zに t[mm] だけ持ち上げる（姿勢はBと同じ）
#  - 先端群:
#       Pb … Bにいる時の先端（赤）， z=O.z（に平均合わせ）
#       Pa … Aにいる時の先端（紫）， z=O.z + t
#  - 表示チェック: A/B/Pb/Pa を個別ON/OFF
#  - push_plan.json 出力: id, A, B, Pa(各N点), Pb(各N点)
#
# 変更点：
#  - GUIで roll/pitch/yaw [deg] を設定可
#  - 手首特異点回避用の微小バイアス roll_bias/pitch_bias [deg] を設定可
#  - RPY（＋バイアス）に基づいた一般回転で先端オフセットとBzを計算

import os, json, math
import numpy as np
from PyQt5 import QtWidgets, QtCore
import matplotlib
matplotlib.use("Qt5Agg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas

HERE = os.path.abspath(os.path.dirname(__file__))
PINART_CANDIDATES = [
    os.path.join(HERE, "pinart_info.json"),
    os.path.join(HERE, "setting", "pinart_info.json"),
]

# ---------------- IO ----------------
def load_pinart_info():
    path = None
    for p in PINART_CANDIDATES:
        if os.path.exists(p):
            path = p; break
    if path is None:
        raise FileNotFoundError(
            "pinart_info.json が見つかりません．\n"
            f"- {PINART_CANDIDATES[0]}\n- {PINART_CANDIDATES[1]}"
        )
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)
    for k in ["num_pins","pin_diameter_mm","pin_length_mm","pin_bases_wrist_xyz_mm"]:
        if k not in data:
            raise KeyError(f"{os.path.basename(path)} に '{k}' がありません．")
    bases = np.array(data["pin_bases_wrist_xyz_mm"], float)  # (N,3) 手先座標系
    N = bases.shape[0]
    d = float(data["pin_diameter_mm"])
    L = float(data["pin_length_mm"])
    return N, d, L, bases, path

# ------------- rotation utils ---------------
def deg2rad(x): return x * math.pi / 180.0

def Rz(y):
    cy, sy = math.cos(y), math.sin(y)
    return np.array([[cy, -sy, 0.0],
                     [sy,  cy, 0.0],
                     [0.0, 0.0, 1.0]], float)

def Ry(p):
    cp, sp = math.cos(p), math.sin(p)
    return np.array([[ cp, 0.0, sp],
                     [0.0, 1.0, 0.0],
                     [-sp, 0.0, cp]], float)

def Rx(r):
    cr, sr = math.cos(r), math.sin(r)
    return np.array([[1.0, 0.0,  0.0],
                     [0.0, cr,  -sr],
                     [0.0, sr,   cr]], float)

def R_orient_zyx(roll_deg, pitch_deg, yaw_deg):
    """
    RAPID OrientZYX と同順序の回転を想定（Z→Y→Xの順に適用）
    R = Rz(yaw) @ Ry(pitch) @ Rx(roll)
    """
    r = deg2rad(roll_deg)
    p = deg2rad(pitch_deg)
    y = deg2rad(yaw_deg)
    return Rz(y) @ Ry(p) @ Rx(r)

# ------------- util ---------------
def build_targets(O, lx, ly, dx, dy):
    xs = np.arange(O[0]-lx/2, O[0]+lx/2+1e-9, dx)
    ys = np.arange(O[1]-ly/2, O[1]+ly/2+1e-9, dy)
    X, Y = np.meshgrid(xs, ys)
    T = np.column_stack([X.ravel(), Y.ravel(), np.full(X.size, O[2], float)])
    return T, xs, ys

def round_key_xy(pt, nd=6):
    return (round(float(pt[0]), nd), round(float(pt[1]), nd))

def set_equal_3d(ax, pts):
    pts = np.asarray(pts)
    if pts.size == 0:
        ax.set_box_aspect([1,1,0.1]); return
    mn = pts.min(axis=0); mx = pts.max(axis=0)
    c = (mn+mx)/2.0; r = (mx-mn).max()/2.0
    r = max(r, 1e-6)
    ax.set_xlim(c[0]-r, c[0]+r)
    ax.set_ylim(c[1]-r, c[1]+r)
    ax.set_zlim(c[2]-r, c[2]+r)

# ------- planner（中心対称の等間隔格子 / 単一点ならB=O） -------
def plan_centered_uniform_grid(O, lx, ly, dx, dy, bases_xyz, L,
                               roll_deg, pitch_deg, yaw_deg,
                               roll_bias_deg, pitch_bias_deg):
    """
    目的:
      - 探索範囲の中心 O=(Ox,Oy,Oz) を幾何学的中心として、
        B（手先中心）を x,y ともに対称かつ等間隔で配置
      - RPYはGUIで指定（roll,pitch,yaw）＋微小バイアス（roll_bias, pitch_bias）を適用
      - Bz は、各ピン先端のワールドzが平均的に O.z に合うよう
        Bz = O.z - mean_i( (R @ [px,py,pz+L])_z )
    戻り値:
      B_xy, Pb_xy_all, Bz, uncovered(空), S_xy, R_used
    """
    # ガード
    if dx <= 0 or dy <= 0 or lx <= 0 or ly <= 0:
        return np.zeros((0,2)), np.zeros((0,2)), float("nan"), np.zeros((0,2)), np.zeros((0,2)), np.eye(3)

    # 使用回転（バイアス適用後）
    r_use = roll_deg + roll_bias_deg
    p_use = pitch_deg + pitch_bias_deg
    y_use = yaw_deg
    R = R_orient_zyx(r_use, p_use, y_use)

    # 各ピン先端の「手先原点からの」ワールド相対オフセット
    # tip_local = base + [0,0,L]
    if bases_xyz.size > 0:
        tip_local = bases_xyz + np.array([0.0, 0.0, float(L)], float)
        tip_off_world = (R @ tip_local.T).T  # (N,3)
        # XYオフセット（B中心に対する先端位置のXY差分）
        S_xy = tip_off_world[:, :2]  # (N,2)
        # Bzを平均合わせ
        mean_tip_z = float(np.mean(tip_off_world[:, 2]))
        Bz = float(O[2] - mean_tip_z)
    else:
        S_xy = np.zeros((0,2))
        Bz = float(O[2])

    # ===== 中心対称の格子点数 nx, ny を決定 =====
    nx = max(1, int(math.floor(lx / dx)) + 1)
    ny = max(1, int(math.floor(ly / dy)) + 1)
    while (nx - 1) * dx > lx + 1e-9 and nx > 1:
        nx -= 1
    while (ny - 1) * dy > ly + 1e-9 and ny > 1:
        ny -= 1

    # ===== 中心対称の座標列を生成 =====
    cx = (nx - 1) / 2.0
    cy = (ny - 1) / 2.0
    xs = O[0] + (np.arange(nx) - cx) * dx
    ys = O[1] + (np.arange(ny) - cy) * dy

    if nx * ny == 1:
        B_xy = np.array([[float(O[0]), float(O[1])]], dtype=float)
    else:
        X, Y = np.meshgrid(xs, ys)
        B_xy = np.column_stack([X.ravel(), Y.ravel()])

    # 表示用：全Bでの Pb（先端XY） = B_xy + S_xy を全点で並べる
    if B_xy.size > 0 and S_xy.size > 0:
        Pb_xy_all = np.vstack([b[None, :] + S_xy for b in B_xy])
    else:
        Pb_xy_all = np.zeros((0,2))

    uncovered = np.zeros((0,2))  # 本仕様では未使用
    return B_xy, Pb_xy_all, Bz, uncovered, S_xy, R

# --------------- GUI ----------------
class PlannerGUI(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("All-Pins Press Planner（RPY指定 & 微小バイアス対応 / 2D⇄3D / A退避付き）")

        try:
            self.N, self.pin_diam, self.pin_len, self.bases_wrist, self.loaded_path = load_pinart_info()
        except Exception as e:
            QtWidgets.QMessageBox.critical(self, "読み込みエラー", str(e))
            self.N, self.pin_diam, self.pin_len = 0, 0.0, 0.0
            self.bases_wrist = np.zeros((0,3), float)
            self.loaded_path = "(none)"

        form = QtWidgets.QFormLayout()
        # O, 長方形, 格子ピッチ
        self.ox=QtWidgets.QDoubleSpinBox(); self.ox.setRange(-1e6,1e6); self.ox.setDecimals(3); self.ox.setValue(500.0)
        self.oy=QtWidgets.QDoubleSpinBox(); self.oy.setRange(-1e6,1e6); self.oy.setDecimals(3); self.oy.setValue(0.0)
        self.oz=QtWidgets.QDoubleSpinBox(); self.oz.setRange(-1e6,1e6); self.oz.setDecimals(3); self.oz.setValue(20.0)
        form.addRow("中心 O.x [mm]", self.ox)
        form.addRow("中心 O.y [mm]", self.oy)
        form.addRow("中心 O.z [mm]", self.oz)

        self.lx=QtWidgets.QDoubleSpinBox(); self.lx.setRange(0.1,1e6); self.lx.setDecimals(3); self.lx.setValue(300.0)
        self.ly=QtWidgets.QDoubleSpinBox(); self.ly.setRange(0.1,1e6); self.ly.setDecimals(3); self.ly.setValue(400.0)
        self.dx=QtWidgets.QDoubleSpinBox(); self.dx.setRange(0.1,1e6); self.dx.setDecimals(6); self.dx.setValue(110.0)
        self.dy=QtWidgets.QDoubleSpinBox(); self.dy.setRange(0.1,1e6); self.dy.setDecimals(6); self.dy.setValue(110.0)
        form.addRow("長方形 辺 lx [mm]", self.lx)
        form.addRow("長方形 辺 ly [mm]", self.ly)
        form.addRow("格子間隔 dx [mm]", self.dx)
        form.addRow("格子間隔 dy [mm]", self.dy)

        # 退避量 t
        self.clearance = QtWidgets.QDoubleSpinBox(); self.clearance.setRange(0.0, 1e6); self.clearance.setDecimals(3); self.clearance.setValue(330.0)
        form.addRow("退避量 t (= A.z - B.z) [mm]", self.clearance)

        # ---- RPY（ユーザ指定）----
        self.roll  = QtWidgets.QDoubleSpinBox();  self.roll.setRange(-360,360);   self.roll.setDecimals(3);  self.roll.setValue(0.0)
        self.pitch = QtWidgets.QDoubleSpinBox();  self.pitch.setRange(-360,360);  self.pitch.setDecimals(3); self.pitch.setValue(180.0)
        self.yaw   = QtWidgets.QDoubleSpinBox();  self.yaw.setRange(-360,360);    self.yaw.setDecimals(3);   self.yaw.setValue(-45.0)
        form.addRow("手先 roll [deg]",  self.roll)
        form.addRow("手先 pitch [deg]", self.pitch)
        form.addRow("手先 yaw [deg]",   self.yaw)

        # ---- 手首特異点回避用 バイアス ----
        self.r_bias = QtWidgets.QDoubleSpinBox(); self.r_bias.setRange(-30,30); self.r_bias.setDecimals(3); self.r_bias.setValue(0.0)
        self.p_bias = QtWidgets.QDoubleSpinBox(); self.p_bias.setRange(-30,30); self.p_bias.setDecimals(3); self.p_bias.setValue(0.0)
        form.addRow("roll バイアス [deg]",  self.r_bias)
        form.addRow("pitch バイアス [deg]", self.p_bias)

        # 情報
        form.addRow(QtWidgets.QLabel(f"読み込み: {os.path.relpath(self.loaded_path, HERE)} / pins={self.N}, L={self.pin_len:.1f} mm"))

        # 表示モード切替（初期2D）
        self.rb2d = QtWidgets.QRadioButton("2D 表示"); self.rb2d.setChecked(True)
        self.rb3d = QtWidgets.QRadioButton("3D 表示")
        hb_mode = QtWidgets.QHBoxLayout(); hb_mode.addWidget(self.rb2d); hb_mode.addWidget(self.rb3d)
        form.addRow("表示モード", hb_mode)

        # 表示チェック
        self.cb_show_A  = QtWidgets.QCheckBox("点A (緑丸)"); self.cb_show_A.setChecked(True)
        self.cb_show_B  = QtWidgets.QCheckBox("点B (青〇)"); self.cb_show_B.setChecked(True)
        self.cb_show_Pb = QtWidgets.QCheckBox("Pb: Bでの先端 (赤丸)"); self.cb_show_Pb.setChecked(True)
        self.cb_show_Pa = QtWidgets.QCheckBox("Pa: Aでの先端 (紫丸)"); self.cb_show_Pa.setChecked(False)
        hb_chk = QtWidgets.QHBoxLayout()
        hb_chk.addWidget(self.cb_show_A); hb_chk.addWidget(self.cb_show_B)
        hb_chk.addWidget(self.cb_show_Pb); hb_chk.addWidget(self.cb_show_Pa)
        form.addRow("表示項目", hb_chk)

        # ボタン
        btns = QtWidgets.QHBoxLayout()
        self.preview_btn = QtWidgets.QPushButton("グラフに反映")
        self.save_plan_btn = QtWidgets.QPushButton("push_plan.json 保存")
        btns.addWidget(self.preview_btn); btns.addWidget(self.save_plan_btn)
        self.preview_btn.clicked.connect(self.update_plot)
        self.save_plan_btn.clicked.connect(self.save_push_plan)

        # Figure（初期2D）
        self.fig = plt.figure(figsize=(8,8))
        self.canvas = FigureCanvas(self.fig)
        self.ax = self.fig.add_subplot(111)

        # レイアウト
        left = QtWidgets.QVBoxLayout()
        left.addLayout(form)
        left.addLayout(btns)
        main = QtWidgets.QHBoxLayout(self)
        main.addLayout(left, 0)
        main.addWidget(self.canvas, 1)

        # 内部キャッシュ
        self.cached_O = None
        self.cached_B_xy = np.zeros((0,2))
        self.cached_Bz = float("nan")
        self.cached_A_xy = np.zeros((0,2))
        self.cached_Az = float("nan")
        self.cached_tips_B_xy = np.zeros((0,2))  # Pb の XY（全回×本数）
        self.cached_offsets_S = np.zeros((0,2))  # (N,2) 先端XYオフセット（RPY適用後）
        self.cached_t = 0.0
        self.cached_RPY_out = (180.0, 180.0, 0.0)  # 保存用（バイアス適用後）
        self.update_plot()

    def params(self):
        O = np.array([self.ox.value(), self.oy.value(), self.oz.value()], float)
        lx, ly = float(self.lx.value()), float(self.ly.value())
        dx, dy = float(self.dx.value()), float(self.dy.value())
        t = float(self.clearance.value())
        roll = float(self.roll.value())
        pitch = float(self.pitch.value())
        yaw = float(self.yaw.value())
        r_bias = float(self.r_bias.value())
        p_bias = float(self.p_bias.value())
        return O, lx, ly, dx, dy, t, roll, pitch, yaw, r_bias, p_bias

    def ensure_axes(self, is3d):
        self.fig.clf()
        self.ax = self.fig.add_subplot(111, projection='3d' if is3d else None)

    def update_plot(self):
        is3d = self.rb3d.isChecked()
        self.ensure_axes(is3d)

        if self.bases_wrist.size == 0:
            self.ax.text(0.5, 0.5, "pinart_info.json をロードできませんでした", ha='center', va='center')
            self.canvas.draw(); return

        O, lx, ly, dx, dy, t, roll, pitch, yaw, r_bias, p_bias = self.params()

        # 計画：BとPb（Bでの先端XY），Bz，S_xy，R
        B_xy, Pb_xy_all, Bz, uncovered, S_xy, R_used = plan_centered_uniform_grid(
            O, lx, ly, dx, dy, self.bases_wrist, self.pin_len,
            roll, pitch, yaw, r_bias, p_bias
        )
        # A の計算：A.xy = B.xy,  A.z = Bz + t
        A_xy = B_xy.copy()
        Az = Bz + t

        # Pa/Pb の3D点群（表示用）
        Pb = None; Pa = None
        if Pb_xy_all.size > 0:
            Pb = np.column_stack([Pb_xy_all, np.full(Pb_xy_all.shape[0], O[2])])
        if B_xy.size > 0 and S_xy.size > 0:
            Pa_list = []
            for a in A_xy:
                Pa_list.append(np.column_stack([a[None,:] + S_xy, np.full(S_xy.shape[0], O[2] + t)]))
            if Pa_list:
                Pa = np.vstack(Pa_list)

        # キャッシュ（保存時に使用）
        self.cached_O = O
        self.cached_B_xy, self.cached_Bz = B_xy, Bz
        self.cached_A_xy, self.cached_Az = A_xy, Az
        self.cached_tips_B_xy = Pb_xy_all
        self.cached_offsets_S = S_xy
        self.cached_t = t
        # 保存用の出力RPY（＝バイアス適用後）
        self.cached_RPY_out = (roll + r_bias, pitch + p_bias, yaw)

        # 描画
        if is3d:
            self.plot_3d(O, lx, ly, dx, dy, B_xy, Bz, A_xy, Az, Pb, Pa, uncovered,
                         rpy_out=self.cached_RPY_out)
        else:
            self.plot_2d(O, lx, ly, dx, dy, B_xy, A_xy, Pb, Pa, Bz, Az, uncovered,
                         rpy_out=self.cached_RPY_out)
        self.canvas.draw()

    # -------- 2D 描画 --------
    def plot_2d(self, O, lx, ly, dx, dy, B_xy, A_xy, Pb, Pa, Bz, Az, uncovered, rpy_out):
        # O・枠・格子
        self.ax.scatter([O[0]],[O[1]], marker='*', s=220, color='gold', edgecolors='k', linewidths=0.8, label='O')
        rect_x=[O[0]-lx/2,O[0]+lx/2,O[0]+lx/2,O[0]-lx/2,O[0]-lx/2]
        rect_y=[O[1]-ly/2,O[1]-ly/2,O[1]+ly/2,O[1]+ly/2,O[1]-ly/2]
        self.ax.plot(rect_x, rect_y, color='gray', linestyle='--', linewidth=1)
        xs = np.arange(O[0]-lx/2, O[0]+lx/2+1e-9, dx)
        ys = np.arange(O[1]-ly/2, O[1]+ly/2+1e-9, dy)
        for x in xs: self.ax.plot([x,x],[ys[0],ys[-1]], color='lightgray', linewidth=0.5, alpha=0.5)
        for y in ys: self.ax.plot([xs[0],xs[-1]],[y,y], color='lightgray', linewidth=0.5, alpha=0.5)

        # Pb（赤○），Pa（紫○）
        if self.cb_show_Pb.isChecked() and Pb is not None and Pb.size>0:
            self.ax.scatter(Pb[:,0], Pb[:,1], facecolors='none', edgecolors='red', s=120, linewidths=1.5, label='Pb: Bでの先端')
        if self.cb_show_Pa.isChecked() and Pa is not None and Pa.size>0:
            self.ax.scatter(Pa[:,0], Pa[:,1], facecolors='none', edgecolors='purple', s=100, linewidths=1.3, label='Pa: Aでの先端')

        # 未カバー格子
        if uncovered.size>0 and self.cb_show_Pb.isChecked():
            self.ax.scatter(uncovered[:,0], uncovered[:,1], marker='x', color='red', s=70, label='未カバー格子')

        # 点B（青），点A（緑）
        if self.cb_show_B.isChecked() and B_xy.size>0:
            self.ax.scatter(B_xy[:,0], B_xy[:,1], color='blue', s=70, label=f'B（回数={B_xy.shape[0]}，z={Bz:.1f}）')
        if self.cb_show_A.isChecked() and A_xy.size>0:
            self.ax.scatter(A_xy[:,0], A_xy[:,1], color='green', s=70, label=f'A（z=Bz+t={Az:.1f}）')

        self.ax.set_aspect('equal', adjustable='box')
        self.ax.grid(True, linestyle=':')
        self.ax.set_xlabel("X [mm]"); self.ax.set_ylabel("Y [mm]")
        rr, pp, yy = rpy_out
        self.ax.set_title(f"全ピン同時押下（重複ゼロ）2D  /  A= B を +Z に t 退避\nRPY=({rr:.2f},{pp:.2f},{yy:.2f}) [deg]")
        self.ax.legend(loc='best')

    # -------- 3D 描画 --------
    def plot_3d(self, O, lx, ly, dx, dy, B_xy, Bz, A_xy, Az, Pb, Pa, uncovered, rpy_out):
        # O平面（半透明）
        Xp=[O[0]-lx/2, O[0]+lx/2]; Yp=[O[1]-ly/2, O[1]+ly/2]
        Xg, Yg = np.meshgrid(Xp, Yp)
        Zg = np.full_like(Xg, O[2], float)
        self.ax.plot_surface(Xg, Yg, Zg, alpha=0.15)

        # グリッド
        xs = np.arange(O[0]-lx/2, O[0]+lx/2+1e-9, dx)
        ys = np.arange(O[1]-ly/2, O[1]+ly/2+1e-9, dy)
        for x in xs: self.ax.plot([x,x],[ys[0],ys[-1]],[O[2],O[2]], color='lightgray', linewidth=0.5, alpha=0.6)
        for y in ys: self.ax.plot([xs[0],xs[-1]],[y,y],[O[2],O[2]], color='lightgray', linewidth=0.5, alpha=0.6)

        # O（黄星）
        self.ax.scatter([O[0]],[O[1]],[O[2]], marker='*', s=200, color='gold', edgecolors='k', linewidths=0.8, label='O')

        # Pb（赤○），Pa（紫○）
        if self.cb_show_Pb.isChecked() and Pb is not None and Pb.size>0:
            self.ax.scatter(Pb[:,0], Pb[:,1], Pb[:,2], edgecolors='red', facecolors='none', s=60, label='Pb: Bでの先端')
        if self.cb_show_Pa.isChecked() and Pa is not None and Pa.size>0:
            self.ax.scatter(Pa[:,0], Pa[:,1], Pa[:,2], edgecolors='purple', facecolors='none', s=50, label='Pa: Aでの先端')

        # 未カバー格子
        if uncovered.size>0 and self.cb_show_Pb.isChecked():
            self.ax.scatter(uncovered[:,0], uncovered[:,1], np.full(uncovered.shape[0], O[2]),
                            marker='x', color='red', s=40, label='未カバー格子')

        # 点B（青）と点A（緑）
        if self.cb_show_B.isChecked() and B_xy.size>0:
            self.ax.scatter(B_xy[:,0], B_xy[:,1], np.full(B_xy.shape[0], Bz), color='blue', s=50, label=f'B（z={Bz:.1f}）')
        if self.cb_show_A.isChecked() and A_xy.size>0:
            self.ax.scatter(A_xy[:,0], A_xy[:,1], np.full(A_xy.shape[0], Az), color='green', s=50, label=f'A（z={Az:.1f}）')

        # A↔B 縦ライン
        if self.cb_show_A.isChecked() and self.cb_show_B.isChecked():
            for a, b in zip(A_xy, B_xy):
                self.ax.plot([a[0], b[0]], [a[1], b[1]], [Az, Bz], color='gray', linewidth=0.8, alpha=0.5)

        # 軸ほか
        self.ax.set_xlabel("X [mm]"); self.ax.set_ylabel("Y [mm]"); self.ax.set_zlabel("Z [mm]")
        pts = [np.array([[O[0],O[1],O[2]]])]
        if Pb is not None and Pb.size>0: pts.append(Pb)
        if Pa is not None and Pa.size>0: pts.append(Pa)
        if B_xy.size > 0: pts.append(np.column_stack([B_xy, np.full(B_xy.shape[0], Bz)]))
        if A_xy.size > 0: pts.append(np.column_stack([A_xy, np.full(A_xy.shape[0], Az)]))
        self.ax.set_title(f"全ピン同時押下（重複ゼロ）3D  /  A= B を +Z に t 退避\nRPY=({rpy_out[0]:.2f},{rpy_out[1]:.2f},{rpy_out[2]:.2f}) [deg]")
        set_equal_3d(self.ax, np.vstack(pts))
        self.ax.legend(loc='upper right')

    # -------- JSON保存 --------
    def save_push_plan(self):
        """A,B,Pa,Pb を push_plan.json に保存（IDは B.x→B.y 昇順）"""
        if self.cached_B_xy.size == 0:
            QtWidgets.QMessageBox.warning(self, "警告", "保存対象がありません．先に『グラフに反映』してください．")
            return

        # 並べ替え: B.x 昇順 → B.y 昇順
        order = np.lexsort((self.cached_B_xy[:,1], self.cached_B_xy[:,0]))
        B_xy_sorted = self.cached_B_xy[order]
        A_xy_sorted = self.cached_A_xy[order]

        # 定数・オフセット
        O  = self.cached_O
        Bz = float(self.cached_Bz)
        Az = float(self.cached_Az)
        t  = float(self.cached_t)
        S  = self.cached_offsets_S  # (N,2)
        r_out, p_out, y_out = self.cached_RPY_out  # バイアス適用後

        out_list = []
        for idx, (Bxy, Axy) in enumerate(zip(B_xy_sorted, A_xy_sorted), start=1):
            # Pb … Bにいる時の先端（z=O.z）
            Pb_xy = (Bxy[None, :] + S)  # (N,2)
            Pb = [[float(x), float(y), float(O[2])] for (x,y) in Pb_xy]

            # Pa … Aにいる時の先端（z=O.z + t）
            Pa_xy = (Axy[None, :] + S)  # (N,2)
            Pa = [[float(x), float(y), float(O[2] + t)] for (x,y) in Pa_xy]

            A_pose = [float(Axy[0]), float(Axy[1]), float(Az), r_out, p_out, y_out]
            B_pose = [float(Bxy[0]), float(Bxy[1]), float(Bz), r_out, p_out, y_out]

            out_list.append({
                "id": int(idx),
                "A": A_pose,
                "B": B_pose,
                "Pa": Pa,   # A時の先端 N点
                "Pb": Pb    # B時の先端 N点
            })

        save_path = os.path.join(HERE, "push_plan.json")
        with open(save_path, "w", encoding="utf-8") as f:
            json.dump({"plan": out_list}, f, indent=2, ensure_ascii=False)

        QtWidgets.QMessageBox.information(
            self, "保存完了",
            f"push_plan.json を保存しました。\n{save_path}\n"
            f"RPY=({r_out:.3f},{p_out:.3f},{y_out:.3f}) [deg]（バイアス適用後）"
        )

def main():
    app = QtWidgets.QApplication([])
    w = PlannerGUI()
    w.resize(1250, 860)
    w.show()
    app.exec_()

if __name__ == "__main__":
    main()
