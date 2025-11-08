# -*- coding: utf-8 -*-
# pinart_gui_matplotlib.py
# matplotlib版 3D円柱描画 + JSON保存（太さが見た目に反映されます）

import os, json, math
import numpy as np
from PyQt5 import QtWidgets
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

def rpy_deg_to_rot(roll, pitch, yaw):
    """RPY(roll->pitch->yaw)[deg] を回転行列に変換"""
    rx, ry, rz = np.radians([roll, pitch, yaw])
    Rx = np.array([[1,0,0],[0,math.cos(rx),-math.sin(rx)],[0,math.sin(rx),math.cos(rx)]])
    Ry = np.array([[math.cos(ry),0,math.sin(ry)],[0,1,0],[-math.sin(ry),0,math.cos(ry)]])
    Rz = np.array([[math.cos(rz),-math.sin(rz),0],[math.sin(rz),math.cos(rz),0],[0,0,1]])
    return Rz @ Ry @ Rx

def make_pin_grid_xy(N, side_l):
    """中心(0,0)の正方形内にN点を均等格子配置（z=0）"""
    if N <= 0:
        return np.zeros((0,3))
    g = math.ceil(math.sqrt(N))
    xs = np.linspace(-side_l/2, side_l/2, g)
    ys = np.linspace(-side_l/2, side_l/2, g)
    pts = []
    for j in range(g):
        for i in range(g):
            if len(pts) >= N: break
            pts.append([xs[i], ys[g-1-j], 0.0])
        if len(pts) >= N: break
    return np.array(pts)

def orthonormal_basis_from_dir(dirw):
    """dirw（単位ベクトル）をz軸とする直交基底を作る"""
    k = dirw / (np.linalg.norm(dirw) + 1e-12)
    # ほぼx軸と平行かどうかで補助ベクトルを選ぶ
    helper = np.array([0.0, 1.0, 0.0]) if abs(np.dot(k, [1,0,0])) > 0.9 else np.array([1.0, 0.0, 0.0])
    i = np.cross(helper, k); i /= (np.linalg.norm(i) + 1e-12)
    j = np.cross(k, i);     j /= (np.linalg.norm(j) + 1e-12)
    # 列が基底ベクトル（ローカル[x,y,z]→ワールドへ）
    return np.column_stack([i, j, k])

def cylinder_mesh(base, dirw, radius, height, n_theta=32, n_z=8):
    """ローカル円柱をdirw方向に回したワールド座標メッシュを返す（側面のみ）
       返り値: X, Y, Z（各 n_z x n_theta）
    """
    dirw = dirw / (np.linalg.norm(dirw) + 1e-12)
    M = orthonormal_basis_from_dir(dirw)  # 3x3
    theta = np.linspace(0, 2*np.pi, n_theta, endpoint=True)
    zgrid = np.linspace(0, height, n_z)
    # ローカル円柱
    Xl = radius * np.cos(theta)[None, :].repeat(n_z, axis=0)     # (n_z, n_theta)
    Yl = radius * np.sin(theta)[None, :].repeat(n_z, axis=0)
    Zl = zgrid[:, None].repeat(n_theta, axis=1)
    # 変換（各点: base + M @ [Xl,Yl,Zl]）
    P = M @ np.stack([Xl, Yl, Zl], axis=0).reshape(3, -1)        # (3, n_z*n_theta)
    P = P.T + base[None, :]                                      # (n_z*n_theta, 3)
    X = P[:,0].reshape(n_z, n_theta)
    Y = P[:,1].reshape(n_z, n_theta)
    Z = P[:,2].reshape(n_z, n_theta)
    return X, Y, Z

def set_equal_3d(ax, pts):
    """3D軸を等比にする"""
    pts = np.asarray(pts)
    xmin, ymin, zmin = pts.min(axis=0)
    xmax, ymax, zmax = pts.max(axis=0)
    cx, cy, cz = (xmax+xmin)/2, (ymax+ymin)/2, (zmax+zmin)/2
    half = max(xmax-xmin, ymax-ymin, zmax-zmin) / 2
    half = max(half, 1e-3)
    ax.set_xlim(cx-half, cx+half)
    ax.set_ylim(cy-half, cy+half)
    ax.set_zlim(cz-half, cz+half)

class PinArtGUI(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Pin Art Sensor (matplotlib円柱版)")

        # 入力欄（初期値: ①9 ②10 ③30, l=60）
        form = QtWidgets.QFormLayout()
        self.spin_count = QtWidgets.QSpinBox(); self.spin_count.setRange(1,10000); self.spin_count.setValue(9)
        self.diam_edit  = QtWidgets.QDoubleSpinBox(); self.diam_edit.setRange(0.1,1000); self.diam_edit.setDecimals(3); self.diam_edit.setValue(10.0)
        self.len_edit   = QtWidgets.QDoubleSpinBox(); self.len_edit.setRange(0.1,2000); self.len_edit.setDecimals(3); self.len_edit.setValue(30.0)
        form.addRow("① ピン数 [本]", self.spin_count)
        form.addRow("② ピン直径 [mm]", self.diam_edit)
        form.addRow("③ ピン長さ [mm]", self.len_edit)

        self.cx=QtWidgets.QDoubleSpinBox(); self.cx.setRange(-10000,10000); self.cx.setDecimals(3); self.cx.setValue(0.0)
        self.cy=QtWidgets.QDoubleSpinBox(); self.cy.setRange(-10000,10000); self.cy.setDecimals(3); self.cy.setValue(0.0)
        self.cz=QtWidgets.QDoubleSpinBox(); self.cz.setRange(-10000,10000); self.cz.setDecimals(3); self.cz.setValue(50.0)
        self.cr=QtWidgets.QDoubleSpinBox(); self.cr.setRange(-180,180); self.cr.setDecimals(3); self.cr.setValue(0.0)
        self.cp=QtWidgets.QDoubleSpinBox(); self.cp.setRange(-180,180); self.cp.setDecimals(3); self.cp.setValue(0.0)
        self.cw=QtWidgets.QDoubleSpinBox(); self.cw.setRange(-180,180); self.cw.setDecimals(3); self.cw.setValue(135.0)
        form.addRow("中心X [mm]", self.cx)
        form.addRow("中心Y [mm]", self.cy)
        form.addRow("中心Z [mm]", self.cz)
        form.addRow("中心Roll [deg]", self.cr)
        form.addRow("中心Pitch [deg]", self.cp)
        form.addRow("中心Yaw [deg]", self.cw)

        self.side_l = QtWidgets.QDoubleSpinBox(); self.side_l.setRange(0.1, 2000); self.side_l.setDecimals(3); self.side_l.setValue(60.0)
        form.addRow("正方形一辺 l [mm]", self.side_l)

        self.preview_btn = QtWidgets.QPushButton("プレビュー更新")
        self.save_btn    = QtWidgets.QPushButton("JSON保存")
        btns = QtWidgets.QHBoxLayout(); btns.addWidget(self.preview_btn); btns.addWidget(self.save_btn)

        # matplotlib Figure
        self.fig = plt.figure(figsize=(6,6))
        self.canvas = FigureCanvas(self.fig)

        left = QtWidgets.QVBoxLayout(); left.addLayout(form); left.addLayout(btns)
        main = QtWidgets.QHBoxLayout(self); main.addLayout(left,0); main.addWidget(self.canvas,1)

        self.preview_btn.clicked.connect(self.update_plot)
        self.save_btn.clicked.connect(self.save_json)

        self.update_plot()

    def params(self):
        return dict(
            N=int(self.spin_count.value()),
            d=float(self.diam_edit.value()),
            L=float(self.len_edit.value()),
            l=float(self.side_l.value()),
            center=np.array([self.cx.value(), self.cy.value(), self.cz.value()], dtype=float),
            rpy=(float(self.cr.value()), float(self.cp.value()), float(self.cw.value()))
        )

    def compute_pins(self):
        p=self.params(); R=rpy_deg_to_rot(*p["rpy"])
        local=make_pin_grid_xy(p["N"], p["l"])
        world=(local@R.T)+p["center"]
        dirw=R@np.array([0.0,0.0,1.0])
        return world, dirw

    def update_plot(self):
        self.fig.clf()
        ax=self.fig.add_subplot(111, projection='3d')
        ax.cla()

        p=self.params()
        pins, dirw = self.compute_pins()
        dirw = dirw / (np.linalg.norm(dirw)+1e-12)
        r = p["d"]/2.0
        L = p["L"]
        l = p["l"]

        # --- 手首座標系のXY平面：灰色の70mm正方形（輪郭） ---
        s = 70.0
        gray = np.array([[-s/2,-s/2,0],[s/2,-s/2,0],[s/2,s/2,0],[-s/2,s/2,0],[-s/2,-s/2,0]])
        ax.plot(gray[:,0], gray[:,1], gray[:,2], color='gray')

        # --- ピンアートXY平面：青のl正方形（姿勢反映） ---
        half = l/2.0
        R = rpy_deg_to_rot(*p["rpy"]); c = p["center"]
        blue_local = np.array([[-half,-half,0],[half,-half,0],[half,half,0],[-half,half,0],[-half,-half,0]])
        blue = (blue_local@R.T)+c
        ax.plot(blue[:,0], blue[:,1], blue[:,2], color='blue')

        # === ラベル用の基底・オフセットを準備（すべてのピンで共通） ===
        M_label = orthonormal_basis_from_dir(dirw)        # 列: [i, j, k=dirw]
        label_offset = (M_label[:,0] + M_label[:,1]) * (r*1.2) + dirw * (L*0.05)

        # --- 各ピンを円柱メッシュで描画（緑） + ラベリング ---
        all_pts = [gray, blue]
        for idx, base in enumerate(pins, start=1):
            X,Y,Z = cylinder_mesh(base, dirw, r, L, n_theta=36, n_z=6)
            ax.plot_surface(X, Y, Z, color='green', linewidth=0, antialiased=False, shade=True)

            # 端面（見た目向上の丸板）
            M = orthonormal_basis_from_dir(dirw)
            theta = np.linspace(0, 2*np.pi, 36, endpoint=True)
            circ_local = np.stack([r*np.cos(theta), r*np.sin(theta), np.full_like(theta, L)], axis=0)
            circ_world = (M @ circ_local).T + base
            poly_top = Poly3DCollection([circ_world], facecolors='green', edgecolors='none', alpha=1.0)
            ax.add_collection3d(poly_top)
            circ_local2 = np.stack([r*np.cos(theta), r*np.sin(theta), np.zeros_like(theta)], axis=0)
            circ_world2 = (M @ circ_local2).T + base
            poly_bot = Poly3DCollection([circ_world2], facecolors='green', edgecolors='none', alpha=1.0)
            ax.add_collection3d(poly_bot)

            # === ここでラベルを配置 ===
            lp = base + label_offset
            ax.text(lp[0], lp[1], lp[2], f"p{idx}",
                    fontsize=9, color='black', ha='center', va='center',
                    bbox=dict(boxstyle='round,pad=0.2', fc='white', ec='none', alpha=0.6))

            all_pts.append(np.column_stack([X.ravel(), Y.ravel(), Z.ravel()]))

        ax.set_xlabel('X [mm]'); ax.set_ylabel('Y [mm]'); ax.set_zlabel('Z [mm]')
        all_pts = np.vstack(all_pts)
        set_equal_3d(ax, all_pts)
        ax.set_title("Pin-Art Sensor Visualization (matplotlib cylinders)")
        self.canvas.draw()

    def save_json(self):
        p=self.params()
        pins,_=self.compute_pins()
        data={
            "num_pins":int(p["N"]),
            "pin_diameter_mm":float(p["d"]),
            "pin_length_mm":float(p["L"]),
            "pin_bases_wrist_xyz_mm":pins.round(6).tolist()
        }
        path=os.path.join(os.path.dirname(__file__),"pinart_info.json")
        with open(path,"w",encoding="utf-8") as f:
            json.dump(data,f,indent=2,ensure_ascii=False)
        QtWidgets.QMessageBox.information(self,"保存完了",f"保存しました:\n{path}")

def main():
    app=QtWidgets.QApplication([])
    w=PinArtGUI(); w.resize(1200,700); w.show()
    app.exec_()

if __name__=="__main__":
    main()
