# -*- coding: utf-8 -*-
# plan_editor.py
# 機能:
#  - pinart_info.json（のみ）を読み込み
#  - 1回の押下で全ピン使用，範囲内格子(dx,dy)を優先して埋める
#  - 先端は重複ゼロを保証
#  - 表示: 2D/3D 切替（初期2D）
#  - 点B: 押下点（青）/ RPY=(0,180,0)固定 / Bzは先端がO.zに来るよう自動
#  - 点A: 退避点（緑）= B をワールド+Zに t[mm] だけ持ち上げる（姿勢はBと同じ）
#  - 先端群:
#       Pb … Bにいる時の先端（赤）， z=O.z
#       Pa … Aにいる時の先端（紫）， z=O.z + t
#  - 表示チェック: A/B/Pb/Pa を個別ON/OFF
#  - push_plan.json 出力: id, A, B, Pa(9点), Pb(9点)

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

# ------------- util ---------------
def build_targets(O, lx, ly, dx, dy):
    xs = np.arange(O[0]-lx/2, O[0]+lx/2+1e-9, dx)
    ys = np.arange(O[1]-ly/2, O[1]+ly/2+1e-9, dy)
    X, Y = np.meshgrid(xs, ys)
    T = np.column_stack([X.ravel(), Y.ravel(), np.full(X.size, O[2], float)])
    return T, xs, ys

def tips_from_Bxy(Bxy, offsets_xy):
    return Bxy[None, :] + offsets_xy  # (N,2)

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

# ------- planner（赤丸重複ゼロ） -------
def plan_without_duplicates(O, lx, ly, dx, dy, bases_xyz, L, tol_nd=6):
    if dx <= 0 or dy <= 0:
        return np.zeros((0,2)), np.zeros((0,2)), float("nan"), np.zeros((0,2)), np.zeros((0,2))

    # 先端 z=O.z に揃う Bz（根元zが全ピン同一前提．違う場合は平均で近似）
    Bz = O[2] + (float(np.mean(bases_xyz[:,2])) + L)

    # RPY=(0,180,0) → 先端XY = B + (-px, +py)
    px, py = bases_xyz[:,0], bases_xyz[:,1]
    S = np.column_stack([-px, +py])  # (N,2) … 先端XYオフセット

    T, xs, ys = build_targets(O, lx, ly, dx, dy)
    Txy = T[:, :2]
    Tkeys = {round_key_xy(q, tol_nd) for q in Txy}
    order = np.lexsort((-Txy[:,0], Txy[:,1]))  # y昇順→x降順

    occupied = set()  # 既出の赤丸（全体）
    covered  = set()  # 範囲内で埋まった格子点
    B_list = []
    tips_all = []

    for idx in order:
        t = Txy[idx]
        tk = round_key_xy(t, tol_nd)
        if tk in covered:
            continue

        best = None
        best_gain = -1
        for i in range(S.shape[0]):
            s_i = S[i]
            Bxy = t - s_i
            tips = tips_from_Bxy(Bxy, S)
            keys = [round_key_xy(p, tol_nd) for p in tips]

            # 既出の赤丸に1つでも一致 → 不採用（重複ゼロ）
            if any(k in occupied for k in keys):
                continue

            gain = sum(1 for k in keys if k in Tkeys and k not in covered)
            if gain > best_gain:
                best_gain = gain
                best = (Bxy, tips, keys)

        if best is None:
            continue  # この格子点は条件下で埋められない

        Bxy, tips, keys = best
        B_list.append(Bxy)
        tips_all.extend(tips.tolist())
        for k in keys:
            occupied.add(k)
            if k in Tkeys:
                covered.add(k)

    # 未カバー格子点（参考）
    uncovered_keys = [k for k in Tkeys if k not in covered]
    uncovered = np.array(uncovered_keys, float) if uncovered_keys else np.zeros((0,2))

    return (
        np.array(B_list, float) if B_list else np.zeros((0,2)),  # B_xy
        np.array(tips_all, float) if tips_all else np.zeros((0,2)),  # Pb_xy (Bでの先端XY)
        Bz,
        uncovered,
        S  # 先端XYオフセット（N,2）
    )

# --------------- GUI ----------------
class PlannerGUI(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("All-Pins Press Planner（重複ゼロ / 2D⇄3D / A退避点付き / Pa・Pb出力）")

        try:
            self.N, self.pin_diam, self.pin_len, self.bases_wrist, self.loaded_path = load_pinart_info()
        except Exception as e:
            QtWidgets.QMessageBox.critical(self, "読み込みエラー", str(e))
            self.N, self.pin_diam, self.pin_len = 0, 0.0, 0.0
            self.bases_wrist = np.zeros((0,3), float)
            self.loaded_path = "(none)"

        form = QtWidgets.QFormLayout()
        # O, 長方形, 格子ピッチ
        self.ox=QtWidgets.QDoubleSpinBox(); self.ox.setRange(-1e6,1e6); self.ox.setDecimals(3); self.ox.setValue(400.0)
        self.oy=QtWidgets.QDoubleSpinBox(); self.oy.setRange(-1e6,1e6); self.oy.setDecimals(3); self.oy.setValue(0.0)
        self.oz=QtWidgets.QDoubleSpinBox(); self.oz.setRange(-1e6,1e6); self.oz.setDecimals(3); self.oz.setValue(400.0)
        form.addRow("中心 O.x [mm]", self.ox)
        form.addRow("中心 O.y [mm]", self.oy)
        form.addRow("中心 O.z [mm]", self.oz)

        self.lx=QtWidgets.QDoubleSpinBox(); self.lx.setRange(0.1,1e6); self.lx.setDecimals(3); self.lx.setValue(60.0)
        self.ly=QtWidgets.QDoubleSpinBox(); self.ly.setRange(0.1,1e6); self.ly.setDecimals(3); self.ly.setValue(60.0)
        self.dx=QtWidgets.QDoubleSpinBox(); self.dx.setRange(0.1,1e6); self.dx.setDecimals(6); self.dx.setValue(10.0)
        self.dy=QtWidgets.QDoubleSpinBox(); self.dy.setRange(0.1,1e6); self.dy.setDecimals(6); self.dy.setValue(10.0)
        form.addRow("長方形 辺 lx [mm]", self.lx)
        form.addRow("長方形 辺 ly [mm]", self.ly)
        form.addRow("格子間隔 dx [mm]", self.dx)
        form.addRow("格子間隔 dy [mm]", self.dy)

        # 退避量 t
        self.clearance = QtWidgets.QDoubleSpinBox(); self.clearance.setRange(0.0, 1e6); self.clearance.setDecimals(3); self.clearance.setValue(300.0)
        form.addRow("退避量 t (= A.z - B.z) [mm]", self.clearance)

        # 姿勢表示
        form.addRow(QtWidgets.QLabel("手先姿勢 roll,pitch,yaw = (0, 180, 0) [deg]（固定）"))
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
        self.cached_offsets_S = np.zeros((0,2))  # (N,2) 先端XYオフセット
        self.cached_t = 0.0
        self.update_plot()

    def params(self):
        O = np.array([self.ox.value(), self.oy.value(), self.oz.value()], float)
        lx, ly = float(self.lx.value()), float(self.ly.value())
        dx, dy = float(self.dx.value()), float(self.dy.value())
        t = float(self.clearance.value())
        return O, lx, ly, dx, dy, t

    def ensure_axes(self, is3d):
        self.fig.clf()
        self.ax = self.fig.add_subplot(111, projection='3d' if is3d else None)

    def update_plot(self):
        is3d = self.rb3d.isChecked()
        self.ensure_axes(is3d)

        if self.bases_wrist.size == 0:
            self.ax.text(0.5, 0.5, "pinart_info.json をロードできませんでした", ha='center', va='center')
            self.canvas.draw(); return

        O, lx, ly, dx, dy, t = self.params()
        # 計画：BとPb（Bでの先端XY），Bz，S
        B_xy, Pb_xy_all, Bz, uncovered, S = plan_without_duplicates(O, lx, ly, dx, dy, self.bases_wrist, self.pin_len)

        # A の計算：A.xy = B.xy,  A.z = Bz + t
        A_xy = B_xy.copy()
        Az = Bz + t

        # Pa（Aでの先端）… XYは A_xy + S， Zは O.z + t
        # Pb（Bでの先端）… XYは B_xy + S， Zは O.z
        # 表示用に平面上の点群を作る
        Pb = None; Pa = None
        if Pb_xy_all.size > 0:
            Pb = np.column_stack([Pb_xy_all, np.full(Pb_xy_all.shape[0], O[2])])
        if B_xy.size > 0 and S.size > 0:
            # A毎の9点をまとめて可視化するため，全部まとめた配列を作る
            Pa_list = []
            for a in A_xy:
                Pa_list.append(np.column_stack([a[None,:] + S, np.full(S.shape[0], O[2] + t)]))
            if Pa_list:
                Pa = np.vstack(Pa_list)

        # キャッシュ（保存時に使用）
        self.cached_O = O
        self.cached_B_xy, self.cached_Bz = B_xy, Bz
        self.cached_A_xy, self.cached_Az = A_xy, Az
        self.cached_tips_B_xy = Pb_xy_all
        self.cached_offsets_S = S  # (N,2)
        self.cached_t = t

        if is3d:
            self.plot_3d(O, lx, ly, dx, dy, B_xy, Bz, A_xy, Az, Pb, Pa, uncovered)
        else:
            self.plot_2d(O, lx, ly, dx, dy, B_xy, A_xy, Pb, Pa, Bz, Az, uncovered)

        self.canvas.draw()

    # -------- 2D 描画 --------
    def plot_2d(self, O, lx, ly, dx, dy, B_xy, A_xy, Pb, Pa, Bz, Az, uncovered):
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

        # 未カバー格子（参考）
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
        self.ax.set_title("全ピン同時押下（重複ゼロ）2D  /  A= B を +Z に t 退避\nRPY=(0,180,0)")
        self.ax.legend(loc='best')

    # -------- 3D 描画 --------
    def plot_3d(self, O, lx, ly, dx, dy, B_xy, Bz, A_xy, Az, Pb, Pa, uncovered):
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

        # A↔B の縦ライン（見やすさ用）
        if self.cb_show_A.isChecked() and self.cb_show_B.isChecked():
            for a, b in zip(A_xy, B_xy):
                self.ax.plot([a[0], b[0]], [a[1], b[1]], [Az, Bz], color='gray', linewidth=0.8, alpha=0.5)

        # 軸ほか
        self.ax.set_xlabel("X [mm]"); self.ax.set_ylabel("Y [mm]"); self.ax.set_zlabel("Z [mm]")
        pts = [np.array([[O[0],O[1],O[2]]])]
        if Pb is not None and Pb.size>0: pts.append(Pb)
        if Pa is not None and Pa.size>0: pts.append(Pa)
        if B_xy.size>0: pts.append(np.column_stack([B_xy, np.full(B_xy.shape[0], Bz)]))
        if A_xy.size>0: pts.append(np.column_stack([A_xy, np.full(A_xy.shape[0], Az)]))
        set_equal_3d(self.ax, np.vstack(pts))
        self.ax.set_title("全ピン同時押下（重複ゼロ）3D  /  A= B を +Z に t 退避\nRPY=(0,180,0)")
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

        roll, pitch, yaw = 0.0, 180.0, 0.0

        out_list = []
        for idx, (Bxy, Axy) in enumerate(zip(B_xy_sorted, A_xy_sorted), start=1):
            # Pb … Bにいる時の先端（z=O.z）
            Pb_xy = (Bxy[None, :] + S)  # (N,2)
            Pb = [[float(x), float(y), float(O[2])] for (x,y) in Pb_xy]

            # Pa … Aにいる時の先端（z=O.z + t）
            Pa_xy = (Axy[None, :] + S)  # (N,2)
            Pa = [[float(x), float(y), float(O[2] + t)] for (x,y) in Pa_xy]

            A_pose = [float(Axy[0]), float(Axy[1]), float(Az), roll, pitch, yaw]
            B_pose = [float(Bxy[0]), float(Bxy[1]), float(Bz), roll, pitch, yaw]

            out_list.append({
                "id": int(idx),
                "A": A_pose,
                "B": B_pose,
                "Pa": Pa,   # A時の先端9点
                "Pb": Pb    # B時の先端9点
            })

        save_path = os.path.join(HERE, "push_plan.json")
        with open(save_path, "w", encoding="utf-8") as f:
            json.dump({"plan": out_list}, f, indent=2, ensure_ascii=False)

        QtWidgets.QMessageBox.information(self, "保存完了", f"push_plan.json を保存しました。\n{save_path}")

def main():
    app = QtWidgets.QApplication([])
    w = PlannerGUI()
    w.resize(1250, 820)
    w.show()
    app.exec_()

if __name__ == "__main__":
    main()
