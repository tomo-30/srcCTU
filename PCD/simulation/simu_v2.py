# -*- coding: utf-8 -*-
# simu.py  ピン太さ対応版：円筒接触（オフセット・レイ）＆球グリフ表示
import json
import os
import sys
import logging
import traceback
import numpy as np

from PyQt5 import QtWidgets, QtCore
import pyvista as pv
from pyvistaqt import QtInteractor

logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")
log = logging.getLogger("simu_pin_radius")

BASE = os.path.dirname(os.path.abspath(sys.argv[0]))

def abspath_from_base(path_text: str) -> str:
    p = path_text.strip()
    if not p:
        return ""
    if not os.path.isabs(p):
        p = os.path.join(BASE, p)
    return os.path.abspath(p)

def segment_plane_intersection(p0, p1, plane_z, eps=1e-9):
    """線分p0->p1と z=plane_z の交差（t,point）．なければ(None,None)."""
    z0, z1 = p0[2], p1[2]
    dz = z1 - z0
    if abs(dz) < eps:
        return None, None
    t = (plane_z - z0) / dz
    if -eps <= t <= 1.0 + eps:
        t = float(np.clip(t, 0.0, 1.0))
        return t, p0 + t * (p1 - p0)
    return None, None

def ortho_basis_from_dir(d):
    """方向ベクトル d (3,) に直交する正規直交基 (u, v) を返す．"""
    d = np.asarray(d, dtype=float)
    n = np.linalg.norm(d)
    if n < 1e-12:
        raise ValueError("方向ベクトルの長さがゼロ")
    d = d / n
    # 直交ベクトルの種
    if abs(d[2]) < 0.9:
        a = np.array([0.0, 0.0, 1.0])
    else:
        a = np.array([1.0, 0.0, 0.0])
    u = np.cross(d, a)
    u /= (np.linalg.norm(u) + 1e-12)
    v = np.cross(d, u)
    v /= (np.linalg.norm(v) + 1e-12)
    return u, v, d

def first_contact_with_radius(mesh, p0, p1, z_ground, radius, n_samples):
    """
    太さ r のピン（円筒）を，中心線 p0->p1 周りのオフセット・レイで近似．
    返り値: (hit_point(3,), type_str["stl"|"ground"], t_min) or (None,None,None)
    """
    p0 = np.asarray(p0, dtype=float)
    p1 = np.asarray(p1, dtype=float)
    seg = p1 - p0
    seg_len = np.linalg.norm(seg)
    if seg_len < 1e-9:
        return None, None, None

    u, v, d_hat = ortho_basis_from_dir(seg)

    # 角度サンプル（中心線も含める）：[0] は半径0（中心線）
    offsets = [np.zeros(3)]
    if radius > 1e-9 and n_samples > 0:
        for k in range(n_samples):
            theta = 2.0 * np.pi * k / n_samples
            off = radius * (np.cos(theta) * u + np.sin(theta) * v)
            offsets.append(off)

    best_t = None
    best_hit = None
    best_type = None

    # STLと地面の「最初のヒット」を各オフセット線分で探索
    for off in offsets:
        q0 = p0 + off
        q1 = p1 + off

        # --- STLヒット ---
        try:
            pts, _ = mesh.ray_trace(q0, q1)
        except Exception as e:
            log.error(f"ray_trace error: {e}")
            pts = []

        if pts is not None and len(pts) > 0:
            hit = np.asarray(pts[0], dtype=float)
            t = np.linalg.norm(hit - q0) / (np.linalg.norm(q1 - q0) + 1e-12)
            if 0.0 <= t <= 1.0:
                if (best_t is None) or (t < best_t):
                    best_t, best_hit, best_type = t, hit, "stl"

        # --- 地面ヒット（オフセット線分と z=ground の交差）---
        tg, hg = segment_plane_intersection(q0, q1, z_ground)
        if tg is not None:
            if (best_t is None) or (tg < best_t):
                best_t, best_hit, best_type = tg, hg, "ground"

    if best_t is None:
        return None, None, None
    return best_hit, best_type, best_t

def compute_contacts_with_radius(mesh, plan_json_path, z_ground, radius, n_samples):
    """push_plan.json を走査し，Pa->Pb 各線分の円筒接触の最初の点を抽出."""
    with open(plan_json_path, "r", encoding="utf-8") as f:
        data = json.load(f)

    contacts = []
    for blk in data.get("plan", []):
        pid = blk.get("id")
        Pa = blk.get("Pa", [])
        Pb = blk.get("Pb", [])
        n = min(len(Pa), len(Pb))
        for i in range(n):
            p0 = np.array(Pa[i], dtype=float)
            p1 = np.array(Pb[i], dtype=float)
            hit, typ, tmin = first_contact_with_radius(mesh, p0, p1, z_ground, radius, n_samples)
            if typ is not None:
                contacts.append({
                    "id": pid,
                    "pin_index": i,
                    "type": typ,
                    "point": list(map(float, hit)),
                    "t": float(tmin)
                })
    return contacts

def decimate_compatible(mesh: pv.PolyData, reduction_frac: float) -> pv.PolyData:
    if not (1e-6 < reduction_frac < 0.95):
        return mesh
    try:
        log.info(f"decimate_pro(target_reduction={reduction_frac:.3f}) を試行")
        return mesh.decimate_pro(target_reduction=float(reduction_frac), preserve_topology=False)
    except TypeError:
        log.info("target_reduction未対応．reduction= を試行")
        try:
            return mesh.decimate_pro(reduction=float(reduction_frac), preserve_topology=False)
        except TypeError:
            log.info("reduction未対応．位置引数を試行")
            return mesh.decimate_pro(float(reduction_frac), preserve_topology=False)

class MainWindow(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("円筒ピン接触シミュレータ（ピン太さ対応）")
        self.resize(1200, 800)

        default_stl  = os.path.join(BASE, "pumpkin1.stl")
        default_json = os.path.join(BASE, "push_plan.json")

        # --- ファイル欄 ---
        self.stl_label = QtWidgets.QLabel("STLファイル:")
        self.stl_path  = QtWidgets.QLineEdit(default_stl if os.path.exists(default_stl) else "pumpkin1.stl")
        self.stl_browse = QtWidgets.QPushButton("参照")

        self.json_label = QtWidgets.QLabel("計画JSON:")
        self.json_path  = QtWidgets.QLineEdit(default_json if os.path.exists(default_json) else "push_plan.json")
        self.json_browse = QtWidgets.QPushButton("参照")

        # --- デシメ ---
        self.decim_check = QtWidgets.QCheckBox("簡略化（デシメーション）を使う")
        self.decim_check.setChecked(True)
        self.decim_label = QtWidgets.QLabel("削減率（%）:")
        self.decim_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal); self.decim_slider.setRange(0, 90); self.decim_slider.setValue(60)
        self.decim_spin   = QtWidgets.QSpinBox(); self.decim_spin.setRange(0, 90); self.decim_spin.setValue(60)
        self.decim_slider.valueChanged.connect(self.decim_spin.setValue)
        self.decim_spin.valueChanged.connect(self.decim_slider.setValue)

        # --- 地面Z / ピン太さ / サンプル数 ---
        self.ground_label = QtWidgets.QLabel("地面Z [mm]:")
        self.ground_edit  = QtWidgets.QLineEdit("0")

        self.radius_label = QtWidgets.QLabel("ピン半径 r [mm]:")
        self.radius_edit  = QtWidgets.QLineEdit("2.0")  # 例: 直径4mmなら r=2

        self.samples_label = QtWidgets.QLabel("円周サンプル数 N:")
        self.samples_spin  = QtWidgets.QSpinBox(); self.samples_spin.setRange(0, 64); self.samples_spin.setValue(12)

        self.generate_btn = QtWidgets.QPushButton("生成（計算＆表示）")
        self.save_simplified_btn = QtWidgets.QPushButton("簡略化STLを保存")

        # --- Plotter ---
        self.plotter = QtInteractor(self); self.plotter.set_background("white")

        # --- レイアウト ---
        form = QtWidgets.QGridLayout(); row = 0
        form.addWidget(self.stl_label,   row,0); form.addWidget(self.stl_path, row,1); form.addWidget(self.stl_browse, row,2); row+=1
        form.addWidget(self.json_label,  row,0); form.addWidget(self.json_path,row,1); form.addWidget(self.json_browse,row,2); row+=1
        form.addWidget(self.decim_check, row,0); row+=1
        form.addWidget(self.decim_label, row,0); form.addWidget(self.decim_slider,row,1); form.addWidget(self.decim_spin,row,2); row+=1
        form.addWidget(self.ground_label,row,0); form.addWidget(self.ground_edit,row,1); row+=1
        form.addWidget(self.radius_label,row,0); form.addWidget(self.radius_edit,row,1); row+=1
        form.addWidget(self.samples_label,row,0); form.addWidget(self.samples_spin,row,1); row+=1
        form.addWidget(self.generate_btn,row,2); row+=1
        form.addWidget(self.save_simplified_btn, row,0,1,3); row+=1

        layout = QtWidgets.QVBoxLayout(self)
        layout.addLayout(form)
        layout.addWidget(self.plotter.interactor)

        # --- 状態 ---
        self.mesh = None
        self.contacts = []
        self.pin_radius = 2.0

        # --- シグナル ---
        self.stl_browse.clicked.connect(self.on_browse_stl)
        self.json_browse.clicked.connect(self.on_browse_json)
        self.generate_btn.clicked.connect(self.on_generate)
        self.save_simplified_btn.clicked.connect(self.on_save_simplified)

    def on_browse_stl(self):
        path, _ = QtWidgets.QFileDialog.getOpenFileName(self, "Select STL", BASE, "STL (*.stl)")
        if path: self.stl_path.setText(os.path.relpath(path, BASE))

    def on_browse_json(self):
        path, _ = QtWidgets.QFileDialog.getOpenFileName(self, "Select JSON", BASE, "JSON (*.json)")
        if path: self.json_path.setText(os.path.relpath(path, BASE))

    def on_generate(self):
        try:
            stl_abs  = abspath_from_base(self.stl_path.text())
            json_abs = abspath_from_base(self.json_path.text())
            z_ground = float(self.ground_edit.text().strip())
            use_decim = self.decim_check.isChecked()
            reduction = self.decim_spin.value() / 100.0
            radius    = float(self.radius_edit.text().strip())
            n_smpl    = int(self.samples_spin.value())

            if radius < 0.0:
                QtWidgets.QMessageBox.warning(self, "入力エラー", "ピン半径は0以上を入力してください．")
                return

            if not os.path.exists(stl_abs):
                QtWidgets.QMessageBox.warning(self, "ファイルなし", f"STLが見つかりません:\n{stl_abs}"); return
            if not os.path.exists(json_abs):
                QtWidgets.QMessageBox.warning(self, "ファイルなし", f"JSONが見つかりません:\n{json_abs}"); return

            log.info(f"STL: {stl_abs}")
            log.info(f"JSON: {json_abs}")
            log.info(f"ground_z: {z_ground}, decimate={use_decim}, reduction={reduction:.2f}, r={radius}, N={n_smpl}")

            # メッシュ読み込み
            mesh = pv.read(stl_abs)
            if not isinstance(mesh, pv.PolyData):
                mesh = mesh.extract_surface()
            mesh = mesh.triangulate().clean(tolerance=0.0)
            if use_decim:
                try:
                    mesh = decimate_compatible(mesh, reduction).clean()
                    log.info("デシメーション完了")
                except Exception as e:
                    log.error(f"デシメーション中にエラー: {e}")
                    log.error(traceback.format_exc())
                    QtWidgets.QMessageBox.warning(self, "簡略化失敗", "デシメ中にエラー．原メッシュで続行します。")

            self.mesh = mesh
            self.pin_radius = radius
            self.contacts = compute_contacts_with_radius(self.mesh, json_abs, z_ground, radius, n_smpl)

            self.refresh_plot_points()

            # 保存
            out_path = os.path.join(BASE, "contact_points_radius.json")
            payload = {
                "ground_z": z_ground,
                "source": {
                    "stl": stl_abs,
                    "push_plan": json_abs,
                    "decimated": use_decim,
                    "target_reduction": reduction
                },
                "pin": {
                    "radius_mm": radius,
                    "samples": n_smpl
                },
                "contacts": self.contacts
            }
            with open(out_path, "w", encoding="utf-8") as f:
                json.dump(payload, f, ensure_ascii=False, indent=2)
            log.info(f"交点を保存しました: {out_path}")
            QtWidgets.QMessageBox.information(self, "保存完了", f"交点を保存しました:\n{out_path}")

        except Exception as e:
            log.error(f"計算失敗: {e}")
            log.error(traceback.format_exc())
            QtWidgets.QMessageBox.critical(self, "計算失敗", str(e))

    def refresh_plot_points(self):
        self.plotter.clear()

        # メッシュも参考表示（半透明）
        if self.mesh is not None:
            self.plotter.add_mesh(self.mesh, color="lightgray", opacity=0.35, show_edges=False)

        if self.contacts:
            pts = np.array([c["point"] for c in self.contacts], dtype=float)
            cloud = pv.PolyData(pts)
            # 半径＝ピン半径の球でグリフ（点サイズではなく実寸で○表示）
            sphere = pv.Sphere(radius=max(self.pin_radius, 1e-6), theta_resolution=20, phi_resolution=20)
            glyphs = cloud.glyph(geom=sphere, scale=False)
            # STL接触は赤，地面接触は青に分ける（2回に分けて描画）
            types = [c["type"] for c in self.contacts]
            stl_mask = np.array([t == "stl" for t in types])
            gnd_mask = ~stl_mask

            if np.any(stl_mask):
                glyphs_stl = pv.PolyData(pts[stl_mask]).glyph(geom=sphere, scale=False)
                self.plotter.add_mesh(glyphs_stl)  # 色指定なし（既定）

            if np.any(gnd_mask):
                glyphs_gnd = pv.PolyData(pts[gnd_mask]).glyph(geom=sphere, scale=False)
                self.plotter.add_mesh(glyphs_gnd)

        self.plotter.reset_camera()

    def on_save_simplified(self):
        if self.mesh is None:
            QtWidgets.QMessageBox.information(self, "未生成", "まず『生成』でメッシュを読み込んでください。")
            return
        path, _ = QtWidgets.QFileDialog.getSaveFileName(self, "簡略化STLを保存", BASE, "STL (*.stl)")
        if not path: return
        try:
            self.mesh.save(path)
            log.info(f"簡略化STLを保存: {path}")
            QtWidgets.QMessageBox.information(self, "保存完了", f"保存しました:\n{path}")
        except Exception as e:
            log.error(f"保存失敗: {e}")
            log.error(traceback.format_exc())
            QtWidgets.QMessageBox.warning(self, "保存失敗", str(e))

if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    w = MainWindow()
    w.show()
    sys.exit(app.exec_())
