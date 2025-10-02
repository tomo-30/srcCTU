# -*- coding: utf-8 -*-
# viewer_gui.py
# PyVista + Qt GUI STL Viewer
# 機能:
#  - 視点リセットボタン: ZY平面を正面, XY平面を水平にする(= +X方向から俯瞰)
#  - バウンディングボックス表示トグル: AABB(軸平行)のワイヤ枠, 寸法表示, 底面中心の赤丸(基準点)表示/非表示
#  - 重いSTLに対して自動デシメーション(削減率指定)

import os
import sys
import pyvista as pv
from pyvistaqt import QtInteractor
from PyQt5 import QtWidgets, QtCore, QtGui

# ===== ユーザ設定 =====
STL_NAME = "pumpkin1_printout.stl"     # 同ディレクトリのSTL名
FACE_THRESHOLD = 100_000      # この面数を超えたらデシメーション実施
TARGET_REDUCTION = 0.90       # デシメーション削減率(0.90=90%削減)
WINDOW_SIZE = (1200, 800)
# =====================


class STLViewer(QtWidgets.QMainWindow):
    def __init__(self, parent=None):
        super().__init__(parent)

        self.setWindowTitle("STL Viewer (PyVista + Qt)")
        self.resize(*WINDOW_SIZE)

        # 中央ウィジェットとレイアウト
        central = QtWidgets.QWidget(self)
        self.setCentralWidget(central)
        layout = QtWidgets.QHBoxLayout(central)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(8)

        # 左: 3Dビュー(QtInteractor)
        self.plotter = QtInteractor(self)
        layout.addWidget(self.plotter, stretch=1)

        # 右: コントロールパネル
        side = QtWidgets.QFrame(self)
        side.setFrameShape(QtWidgets.QFrame.StyledPanel)
        side_layout = QtWidgets.QVBoxLayout(side)
        side_layout.setContentsMargins(8, 8, 8, 8)
        side_layout.setSpacing(10)
        layout.addWidget(side, stretch=0)

        # ラベル: ファイル名・面数等
        self.info_label = QtWidgets.QLabel(self)
        self.info_label.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        self.info_label.setStyleSheet("font-family: Consolas, 'Courier New', monospace;")
        side_layout.addWidget(self.info_label)

        # 視点リセットボタン
        self.btn_reset = QtWidgets.QPushButton("視点リセット (ZY正面 / XY水平)", self)
        self.btn_reset.clicked.connect(self.reset_view)
        side_layout.addWidget(self.btn_reset)

        # バウンディングボックス表示チェック
        self.chk_bbox = QtWidgets.QCheckBox("バウンディングボックスを表示", self)
        self.chk_bbox.stateChanged.connect(self.on_bbox_toggled)
        side_layout.addWidget(self.chk_bbox)

        # 寸法表示用テキスト(画面外でも確認できるようGUI側にも表示)
        self.dim_text = QtWidgets.QTextEdit(self)
        self.dim_text.setReadOnly(True)
        self.dim_text.setPlaceholderText("バウンディングボックス情報を表示する．")
        self.dim_text.setFixedHeight(160)
        side_layout.addWidget(self.dim_text)

        side_layout.addStretch(1)

        # 状態管理
        self.mesh = None
        self.bbox_actor = None
        self.center_actor = None
        self.scene_text_actor = None

        # 読み込み & 初期描画
        self.load_mesh_and_setup()

    # ---------- メッシュ読み込みと描画初期化 ----------
    def load_mesh_and_setup(self):
        current_dir = os.path.dirname(os.path.abspath(__file__))
        stl_path = os.path.join(current_dir, STL_NAME)
        if not os.path.exists(stl_path):
            QtWidgets.QMessageBox.critical(self, "エラー", f"STLが見つからない: {stl_path}")
            sys.exit(1)

        mesh = pv.read(stl_path)
        n_faces = mesh.n_cells
        n_pts = mesh.n_points

        decimated = False
        if n_faces > FACE_THRESHOLD:
            mesh = mesh.decimate(target_reduction=TARGET_REDUCTION)
            decimated = True

        self.mesh = mesh

        # 3Dシーン初期化
        self.plotter.clear()  # 念のため
        self.plotter.add_mesh(self.mesh, show_edges=False, smooth_shading=True)
        self.plotter.add_axes()
        self.plotter.show_grid()
        self.reset_view()  # 初期視点

        # 情報ラベル更新
        info_lines = [
            f"PyVista: {pv.__version__}",
            f"ファイル: {STL_NAME}",
            f"Faces(before): {n_faces:,} / Points(before): {n_pts:,}",
            f"Decimated: {'Yes' if decimated else 'No'}",
            f"Faces(now): {self.mesh.n_cells:,} / Points(now): {self.mesh.n_points:,}",
        ]
        self.info_label.setText("\n".join(info_lines))

    # ---------- 視点リセット ----------
    def reset_view(self):
        # +X方向からのビュー(= ZY平面が正面)，かつXYが水平
        self.plotter.view_yz()      # +Xから見た視点
        self.plotter.reset_camera() # 全体が収まるよう距離調整
        self.plotter.render()

    # ---------- バウンディングボックスのトグル ----------
    def on_bbox_toggled(self, state):
        on = (state == QtCore.Qt.Checked)
        self.toggle_bbox(on)

    def toggle_bbox(self, on: bool):
        # 既存の表示を掃除
        self.remove_bbox_actors()

        if not on:
            self.dim_text.clear()
            self.plotter.render()
            return

        # AABB算出
        xmin, xmax, ymin, ymax, zmin, zmax = self.mesh.bounds
        x_len = xmax - xmin
        y_len = ymax - ymin
        z_len = zmax - zmin

        # ワイヤ枠ボックス
        box = pv.Box(bounds=(xmin, xmax, ymin, ymax, zmin, zmax))
        self.bbox_actor = self.plotter.add_mesh(
            box, style="wireframe", line_width=2.0, color="blue", opacity=1.0
        )

        # 底面中心(基準点)
        cx = (xmin + xmax) / 2.0
        cy = (ymin + ymax) / 2.0
        cz = zmin
        radius = max(x_len, y_len, z_len) * 0.01
        sphere = pv.Sphere(radius=radius, center=(cx, cy, cz))
        self.center_actor = self.plotter.add_mesh(sphere, color="red")

        # シーン内テキスト(右上)
        info = (
            f"Bounding Box size:\n"
            f"  x: {x_len:.3f}\n"
            f"  y: {y_len:.3f}\n"
            f"  z: {z_len:.3f}\n\n"
            f"Reference point (bottom center):\n"
            f"  ({cx:.3f}, {cy:.3f}, {cz:.3f})"
        )
        self.scene_text_actor = self.plotter.add_text(
            info, position="upper_right", font_size=12, color="white"
        )

        # GUI側にも同じ情報を表示
        self.dim_text.setPlainText(info)

        self.plotter.render()

    def remove_bbox_actors(self):
        # 3Dシーンから要素を安全に削除
        for attr in ("bbox_actor", "center_actor", "scene_text_actor"):
            actor = getattr(self, attr, None)
            if actor is not None:
                try:
                    self.plotter.remove_actor(actor)
                except Exception:
                    pass
                setattr(self, attr, None)

    # ---------- 終了処理 ----------
    def closeEvent(self, event: QtGui.QCloseEvent) -> None:
        try:
            self.plotter.close()
        except Exception:
            pass
        return super().closeEvent(event)


def main():
    app = QtWidgets.QApplication(sys.argv)
    win = STLViewer()
    win.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
