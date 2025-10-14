# -*- coding: utf-8 -*-
# stl_pin_contact_gui.py  点群のみ表示版（STL/地面いずれか最初の接触点を採用）
import json
import os
import sys
import logging
import traceback
import numpy as np

from PyQt5 import QtWidgets, QtCore
import pyvista as pv
from pyvistaqt import QtInteractor

# ---- ログ：端末に出力 ----
logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")
log = logging.getLogger("stl_pin_gui")

# 実行ファイルと同じ場所を基準
BASE = os.path.dirname(os.path.abspath(sys.argv[0]))

def abspath_from_base(path_text: str) -> str:
    p = path_text.strip()
    if not p:
        return ""
    if not os.path.isabs(p):
        p = os.path.join(BASE, p)
    return os.path.abspath(p)

def segment_ground_intersection(p0, p1, z_ground, eps=1e-9):
    """線分p0->p1がz=z_groundと交わるなら (hit_point, t) を返す（tは0..1）．なければ(None, None)"""
    z0, z1 = p0[2], p1[2]
    dz = z1 - z0
    if abs(dz) < eps:
        # 水平（z一定）の場合：ちょうど面上を滑るのは採用しない
        return None, None
    t = (z_ground - z0) / dz
    if -eps <= t <= 1+eps:
        t = max(0.0, min(1.0, float(t)))
        hit = p0 + t * (p1 - p0)
        return hit, t
    return None, None

def first_contact_point(mesh, p0, p1, z_ground):
    """
    Pa=p0 から Pb=p1 へ下ろす線分で，STLと地面のうち"先に"当たる点を返す．
    戻り: (point(np.array shape(3,)), type_str["stl"|"ground"]) または (None, None)
    """
    # STL側：ray_traceはスタートから近い順に返す
    pts, _ = mesh.ray_trace(p0, p1)
    t_stl = None
    hit_stl = None
    if pts is not None and len(pts) > 0:
        hit_stl = np.array(pts[0], dtype=float)
        seg_len = np.linalg.norm(p1 - p0)
        if seg_len > 1e-9:
            t_stl = np.linalg.norm(hit_stl - p0) / seg_len

    # 地面側
    hit_g, t_g = segment_ground_intersection(p0, p1, z_ground)

    # どちらも無し
    if t_stl is None and t_g is None:
        return None, None

    # 片方のみ
    if t_stl is None:
        return hit_g, "ground"
    if t_g is None:
        return hit_stl, "stl"

    # 近い方（t小）＝先に当たる方
    if t_g <= t_stl:
        return hit_g, "ground"
    else:
        return hit_stl, "stl"

def compute_contacts(mesh, plan_json_path, z_ground):
    """push_plan.json を走査し，Pa->Pb 各線分の最初の接触点(STL or 地面)を抽出."""
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
            hit, typ = first_contact_point(mesh, p0, p1, z_ground)
            if typ is not None:
                contacts.append({
                    "id": pid,
                    "pin_index": i,
                    "type": typ,                 # "stl" or "ground"
                    "point": list(map(float, hit))
                })
    return contacts

def decimate_compatible(mesh: pv.PolyData, reduction_frac: float) -> pv.PolyData:
    """PyVista差異に耐える decimate_pro 呼び出し．reduction_frac=0.6→60%削減."""
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
        self.setWindowTitle("接触点生成GUI（点群のみ表示）")
        self.resize(1100, 780)

        default_stl  = os.path.join(BASE, "pumpkin1.stl")
        default_json = os.path.join(BASE, "push_plan.json")

        # --- ファイル欄（BASE基準） ---
        self.stl_label = QtWidgets.QLabel("STLファイル:")
        self.stl_path  = QtWidgets.QLineEdit(default_stl if os.path.exists(default_stl) else "pumpkin1.stl")
        self.stl_browse = QtWidgets.QPushButton("参照")

        self.json_label = QtWidgets.QLabel("計画JSON:")
        self.json_path  = QtWidgets.QLineEdit(default_json if os.path.exists(default_json) else "push_plan.json")
        self.json_browse = QtWidgets.QPushButton("参照")

        # --- 簡略化 ---
        self.decim_check = QtWidgets.QCheckBox("簡略化を使う（デシメーション）")
        self.decim_check.setChecked(True)
        self.decim_label = QtWidgets.QLabel("削減率（%）:")
        self.decim_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal); self.decim_slider.setRange(0, 90); self.decim_slider.setValue(60)
        self.decim_spin   = QtWidgets.QSpinBox(); self.decim_spin.setRange(0, 90); self.decim_spin.setValue(60)
        self.decim_slider.valueChanged.connect(self.decim_spin.setValue)
        self.decim_spin.valueChanged.connect(self.decim_slider.setValue)

        # --- 地面Z・実行 ---
        self.ground_label = QtWidgets.QLabel("地面Z [mm]:")
        self.ground_edit  = QtWidgets.QLineEdit("0")
        self.generate_btn = QtWidgets.QPushButton("生成（計算＆表示）")
        self.save_simplified_btn = QtWidgets.QPushButton("簡略化STLを保存")

        # --- Plotter（点群のみ描画） ---
        self.plotter = QtInteractor(self); self.plotter.set_background("white")

        # --- レイアウト ---
        form = QtWidgets.QGridLayout(); row = 0
        form.addWidget(self.stl_label,   row,0); form.addWidget(self.stl_path, row,1); form.addWidget(self.stl_browse, row,2); row+=1
        form.addWidget(self.json_label,  row,0); form.addWidget(self.json_path,row,1); form.addWidget(self.json_browse,row,2); row+=1
        form.addWidget(self.decim_check, row,0); row+=1
        form.addWidget(self.decim_label, row,0); form.addWidget(self.decim_slider,row,1); form.addWidget(self.decim_spin,row,2); row+=1
        form.addWidget(self.ground_label,row,0); form.addWidget(self.ground_edit,row,1); form.addWidget(self.generate_btn,row,2); row+=1
        form.addWidget(self.save_simplified_btn, row,0,1,3); row+=1

        layout = QtWidgets.QVBoxLayout(self)
        layout.addLayout(form)
        layout.addWidget(self.plotter.interactor)

        # --- 状態 ---
        self.mesh = None          # 計算に使う（簡略化後）メッシュ
        self.contacts = []        # 表示・保存する接触点
        # --- シグナル ---
        self.stl_browse.clicked.connect(self.on_browse_stl)
        self.json_browse.clicked.connect(self.on_browse_json)
        self.generate_btn.clicked.connect(self.on_generate)
        self.save_simplified_btn.clicked.connect(self.on_save_simplified)

    # === ファイルUI ===
    def on_browse_stl(self):
        path, _ = QtWidgets.QFileDialog.getOpenFileName(self, "Select STL", BASE, "STL (*.stl)")
        if path: self.stl_path.setText(os.path.relpath(path, BASE))

    def on_browse_json(self):
        path, _ = QtWidgets.QFileDialog.getOpenFileName(self, "Select JSON", BASE, "JSON (*.json)")
        if path: self.json_path.setText(os.path.relpath(path, BASE))

    # === 実行 ===
    def on_generate(self):
        try:
            stl_abs  = abspath_from_base(self.stl_path.text())
            json_abs = abspath_from_base(self.json_path.text())
            z_ground = float(self.ground_edit.text().strip())
            use_decim = self.decim_check.isChecked()
            reduction = self.decim_spin.value() / 100.0

            if not os.path.exists(stl_abs):
                QtWidgets.QMessageBox.warning(self, "ファイルなし", f"STLが見つかりません:\n{stl_abs}"); return
            if not os.path.exists(json_abs):
                QtWidgets.QMessageBox.warning(self, "ファイルなし", f"JSONが見つかりません:\n{json_abs}"); return

            log.info(f"STL: {stl_abs}")
            log.info(f"JSON: {json_abs}")
            log.info(f"ground_z: {z_ground}, decimate={use_decim}, reduction={reduction:.2f}")

            # メッシュ読み込み（表面→三角化→clean→任意でデシメ）
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
                    QtWidgets.QMessageBox.warning(self, "簡略化失敗", "デシメ中にエラーが発生．原メッシュで続行します。")

            self.mesh = mesh
            self.contacts = compute_contacts(self.mesh, json_abs, z_ground)

            # 点群のみ再描画
            self.refresh_plot_points()

            # 保存
            out_path = os.path.join(BASE, "contact_points.json")
            payload = {
                "ground_z": z_ground,
                "source": {
                    "stl": stl_abs,
                    "push_plan": json_abs,
                    "decimated": use_decim,
                    "target_reduction": reduction
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
        if self.contacts:
            pts = np.array([c["point"] for c in self.contacts])
            cloud = pv.PolyData(pts)
            self.plotter.add_mesh(cloud, render_points_as_spheres=True, point_size=12)
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
