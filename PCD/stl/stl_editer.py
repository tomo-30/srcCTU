# -*- coding: utf-8 -*-
# STL Viewer with GUI (PyVista + Qt)
# - 拡大縮小（各軸）
# - 移動（基準点を指定座標へ）
# - 回転（RPY, 基準点まわり）   ← 追加
# - バウンディングボックス表示
# - STL保存
# - 拡大縮小/移動/回転リセット機能

import os
import sys
import numpy as np
import pyvista as pv
from pyvistaqt import QtInteractor
from PyQt5 import QtWidgets, QtCore, QtGui

STL_NAME = "pumpkin_down_on_desk(500-0-67_1-180-0--90).stl"
FACE_THRESHOLD = 100_000
TARGET_REDUCTION = 0.90
WINDOW_SIZE = (1200, 820)
BBOX_COLOR = "blue"


class STLViewer(QtWidgets.QMainWindow):
    def __init__(self, parent=None):
        super().__init__(parent)

        self.setWindowTitle("STL Viewer (PyVista + Qt)")
        self.resize(*WINDOW_SIZE)

        # --- central layout ---
        central = QtWidgets.QWidget(self)
        self.setCentralWidget(central)
        layout = QtWidgets.QHBoxLayout(central)

        # left: 3D view
        self.plotter = QtInteractor(self)
        layout.addWidget(self.plotter, stretch=1)

        # right: control panel
        side = QtWidgets.QFrame(self)
        side.setFrameShape(QtWidgets.QFrame.StyledPanel)
        side_layout = QtWidgets.QVBoxLayout(side)
        layout.addWidget(side, stretch=0)

        # info label
        self.info_label = QtWidgets.QLabel(self)
        self.info_label.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        side_layout.addWidget(self.info_label)

        # reset view
        btn_reset = QtWidgets.QPushButton("視点リセット (ZY正面 / XY水平)")
        btn_reset.clicked.connect(self.reset_view)
        side_layout.addWidget(btn_reset)

        # bbox
        self.chk_bbox = QtWidgets.QCheckBox("バウンディングボックスを表示")
        self.chk_bbox.stateChanged.connect(self.on_bbox_toggled)
        side_layout.addWidget(self.chk_bbox)

        self.dim_text = QtWidgets.QTextEdit(self)
        self.dim_text.setReadOnly(True)
        self.dim_text.setFixedHeight(150)
        side_layout.addWidget(self.dim_text)

        # ---- scale ----
        scale_group = QtWidgets.QGroupBox("拡大縮小（各軸，基準点まわり）")
        g1 = QtWidgets.QGridLayout(scale_group)
        self.sx = self._spin(1.0, 0.01, 100.0, 0.01)
        self.sy = self._spin(1.0, 0.01, 100.0, 0.01)
        self.sz = self._spin(1.0, 0.01, 100.0, 0.01)
        g1.addWidget(QtWidgets.QLabel("sx:"), 0, 0); g1.addWidget(self.sx, 0, 1)
        g1.addWidget(QtWidgets.QLabel("sy:"), 1, 0); g1.addWidget(self.sy, 1, 1)
        g1.addWidget(QtWidgets.QLabel("sz:"), 2, 0); g1.addWidget(self.sz, 2, 1)
        btn_apply_scale = QtWidgets.QPushButton("拡大縮小を適用")
        btn_apply_scale.clicked.connect(self.apply_scale_about_ref)
        g1.addWidget(btn_apply_scale, 3, 0, 1, 2)
        btn_reset_scale = QtWidgets.QPushButton("拡大縮小リセット（形状を元に戻す）")
        btn_reset_scale.clicked.connect(self.reset_scale)
        g1.addWidget(btn_reset_scale, 4, 0, 1, 2)
        side_layout.addWidget(scale_group)

        # ---- move ----
        move_group = QtWidgets.QGroupBox("移動（基準点を指定座標へ）")
        g2 = QtWidgets.QGridLayout(move_group)
        self.tx = self._spin(0.0, -1e6, 1e6, 0.1)
        self.ty = self._spin(0.0, -1e6, 1e6, 0.1)
        self.tz = self._spin(0.0, -1e6, 1e6, 0.1)
        g2.addWidget(QtWidgets.QLabel("x:"), 0, 0); g2.addWidget(self.tx, 0, 1)
        g2.addWidget(QtWidgets.QLabel("y:"), 1, 0); g2.addWidget(self.ty, 1, 1)
        g2.addWidget(QtWidgets.QLabel("z:"), 2, 0); g2.addWidget(self.tz, 2, 1)
        btn_move = QtWidgets.QPushButton("基準点をこの座標へ移動")
        btn_move.clicked.connect(self.move_reference_to_target)
        g2.addWidget(btn_move, 3, 0, 1, 2)
        btn_reset_move = QtWidgets.QPushButton("移動リセット（形状を元に戻す）")
        btn_reset_move.clicked.connect(self.reset_move)
        g2.addWidget(btn_reset_move, 4, 0, 1, 2)
        side_layout.addWidget(move_group)

        # ---- rotation (NEW) ----
        rot_group = QtWidgets.QGroupBox("回転（RPY度，基準点まわり / 適用順: Yaw→Pitch→Roll）")
        g3 = QtWidgets.QGridLayout(rot_group)
        self.r_roll  = self._spin(0.0, -1e6, 1e6, 0.1)    # X
        self.r_pitch = self._spin(0.0, -1e6, 1e6, 0.1)    # Y
        self.r_yaw   = self._spin(0.0, -1e6, 1e6, 0.1)    # Z
        g3.addWidget(QtWidgets.QLabel("Roll (X) [deg]:"), 0, 0); g3.addWidget(self.r_roll, 0, 1)
        g3.addWidget(QtWidgets.QLabel("Pitch (Y) [deg]:"), 1, 0); g3.addWidget(self.r_pitch, 1, 1)
        g3.addWidget(QtWidgets.QLabel("Yaw (Z) [deg]:"), 2, 0); g3.addWidget(self.r_yaw, 2, 1)
        btn_apply_rot = QtWidgets.QPushButton("回転を適用（基準点まわり）")
        btn_apply_rot.clicked.connect(self.apply_rotation_about_ref)
        g3.addWidget(btn_apply_rot, 3, 0, 1, 2)
        btn_reset_rot = QtWidgets.QPushButton("回転リセット（形状を元に戻す）")
        btn_reset_rot.clicked.connect(self.reset_rotation)
        g3.addWidget(btn_reset_rot, 4, 0, 1, 2)
        side_layout.addWidget(rot_group)

        # save
        btn_save = QtWidgets.QPushButton("STLとして保存…")
        btn_save.clicked.connect(self.save_as_stl)
        side_layout.addWidget(btn_save)

        side_layout.addStretch(1)

        # state
        self.mesh = None
        self.mesh_original = None
        self.bbox_actor = None
        self.center_actor = None
        self.scene_text_actor = None

        # load
        self.load_mesh_and_setup()

    # --------- utils ----------
    def _spin(self, vdef, vmin, vmax, step):
        sp = QtWidgets.QDoubleSpinBox()
        sp.setDecimals(6)
        sp.setRange(vmin, vmax)
        sp.setValue(vdef)
        sp.setSingleStep(step)
        return sp

    def _reference_point(self):
        xmin, xmax, ymin, ymax, zmin, zmax = self.mesh.bounds
        return np.array([(xmin+xmax)/2, (ymin+ymax)/2, zmin])

    def _apply_transform(self, M4):
        self.mesh.transform(M4, inplace=True)
        self.refresh_scene(preserve_bbox=self.chk_bbox.isChecked())

    @staticmethod
    def _rpy_deg_to_matrix(roll_deg: float, pitch_deg: float, yaw_deg: float) -> np.ndarray:
        """回転行列（R = Rz(yaw) @ Ry(pitch) @ Rx(roll)）"""
        rx, ry, rz = np.radians([roll_deg, pitch_deg, yaw_deg])
        Rx = np.array([[1, 0, 0],
                       [0, np.cos(rx), -np.sin(rx)],
                       [0, np.sin(rx),  np.cos(rx)]], dtype=float)
        Ry = np.array([[ np.cos(ry), 0, np.sin(ry)],
                       [0,           1, 0],
                       [-np.sin(ry), 0, np.cos(ry)]], dtype=float)
        Rz = np.array([[np.cos(rz), -np.sin(rz), 0],
                       [np.sin(rz),  np.cos(rz), 0],
                       [0,           0,          1]], dtype=float)
        R = Rz @ Ry @ Rx
        M = np.eye(4)
        M[:3, :3] = R
        return M

    def refresh_scene(self, preserve_bbox=False):
        self.plotter.clear()
        self.plotter.add_mesh(self.mesh, show_edges=False, smooth_shading=True)
        self.plotter.add_axes()
        self.plotter.show_grid()
        self.reset_view()
        if preserve_bbox:
            self._show_bbox()
        else:
            self.dim_text.clear()
        info = f"Faces: {self.mesh.n_cells:,}, Points: {self.mesh.n_points:,}"
        self.info_label.setText(info)

    # --------- load/view ----------
    def load_mesh_and_setup(self):
        path = os.path.join(os.path.dirname(__file__), STL_NAME)
        mesh = pv.read(path)
        if mesh.n_cells > FACE_THRESHOLD:
            mesh = mesh.decimate(target_reduction=TARGET_REDUCTION)
        self.mesh = mesh
        self.mesh_original = mesh.copy(deep=True)
        self.refresh_scene()

    def reset_view(self):
        self.plotter.view_yz()
        self.plotter.reset_camera()
        self.plotter.render()

    # --------- bbox ----------
    def on_bbox_toggled(self, state):
        if state == QtCore.Qt.Checked:
            self._show_bbox()
        else:
            self._clear_bbox()

    def _show_bbox(self):
        self._clear_bbox()
        xmin, xmax, ymin, ymax, zmin, zmax = self.mesh.bounds
        box = pv.Box(bounds=(xmin, xmax, ymin, ymax, zmin, zmax))
        self.bbox_actor = self.plotter.add_mesh(box, style="wireframe", line_width=2.0, color=BBOX_COLOR)
        ref = self._reference_point()
        sphere = pv.Sphere(radius=max(xmax-xmin, ymax-ymin, zmax-zmin)*0.01, center=ref)
        self.center_actor = self.plotter.add_mesh(sphere, color="red")
        info = f"x: {xmax-xmin:.3f}, y: {ymax-ymin:.3f}, z: {zmax-zmin:.3f}\nRef: {ref}"
        self.scene_text_actor = self.plotter.add_text(info, position="upper_right")
        self.dim_text.setPlainText(info)
        self.plotter.render()

    def _clear_bbox(self):
        for attr in ("bbox_actor", "center_actor", "scene_text_actor"):
            actor = getattr(self, attr, None)
            if actor:
                try:
                    self.plotter.remove_actor(actor)
                except:
                    pass
                setattr(self, attr, None)
        self.plotter.render()

    # --------- scale ----------
    def apply_scale_about_ref(self):
        sx, sy, sz = self.sx.value(), self.sy.value(), self.sz.value()
        origin = self._reference_point()
        Tm = np.eye(4); Tm[:3, 3] = -origin
        S = np.diag([sx, sy, sz, 1])
        Tp = np.eye(4); Tp[:3, 3] = origin
        self._apply_transform(Tp @ S @ Tm)

    def reset_scale(self):
        if self.mesh_original is not None:
            self.mesh = self.mesh_original.copy(deep=True)
            self.refresh_scene(preserve_bbox=self.chk_bbox.isChecked())
            self.sx.setValue(1.0); self.sy.setValue(1.0); self.sz.setValue(1.0)

    # --------- move ----------
    def move_reference_to_target(self):
        target = np.array([self.tx.value(), self.ty.value(), self.tz.value()])
        delta = target - self._reference_point()
        M = np.eye(4); M[:3, 3] = delta
        self._apply_transform(M)

    def reset_move(self):
        if self.mesh_original is not None:
            self.mesh = self.mesh_original.copy(deep=True)
            self.refresh_scene()
            self.tx.setValue(0.0); self.ty.setValue(0.0); self.tz.setValue(0.0)

    # --------- rotation (NEW) ----------
    def apply_rotation_about_ref(self):
        roll = self.r_roll.value()
        pitch = self.r_pitch.value()
        yaw = self.r_yaw.value()
        R = self._rpy_deg_to_matrix(roll, pitch, yaw)  # Rz@Ry@Rx
        origin = self._reference_point()
        Tm = np.eye(4); Tm[:3, 3] = -origin
        Tp = np.eye(4); Tp[:3, 3] = origin
        self._apply_transform(Tp @ R @ Tm)

    def reset_rotation(self):
        if self.mesh_original is not None:
            self.mesh = self.mesh_original.copy(deep=True)
            self.refresh_scene()
            self.r_roll.setValue(0.0)
            self.r_pitch.setValue(0.0)
            self.r_yaw.setValue(0.0)

    # --------- save ----------
    def save_as_stl(self):
        path, _ = QtWidgets.QFileDialog.getSaveFileName(self, "STL保存", "", "STL files (*.stl)")
        if not path:
            return
        self.mesh.save(path)
        QtWidgets.QMessageBox.information(self, "保存", f"保存しました:\n{path}")


def main():
    app = QtWidgets.QApplication(sys.argv)
    win = STLViewer(); win.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
