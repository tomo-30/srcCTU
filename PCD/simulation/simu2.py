# -*- coding: utf-8 -*-
# stl_pin_contact_gui.py
# 表示：接触点群のみ（STL/地面は描画しない）
# モード：①理想（無制限） ②現実（ピン長制約）
# 追加：
#  - 現実モードで「届かなかったピン」の理想接触点（STL/地面の双方）を赤点で表示 → 赤点数＝×数
#  - 青点を濃い青に
#  - 統計（①〜⑦）をGUIに表示

import json, os, sys, logging, traceback
import numpy as np
from PyQt5 import QtWidgets, QtCore
import pyvista as pv
from pyvistaqt import QtInteractor

logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")
log = logging.getLogger("stl_pin_gui")

BASE = os.path.dirname(os.path.abspath(sys.argv[0]))

def abspath_from_base(path_text: str) -> str:
    p = path_text.strip()
    if not p:
        return ""
    if not os.path.isabs(p):
        p = os.path.join(BASE, p)
    return os.path.abspath(p)

# ---------- 幾何 ----------
def segment_ground_intersection(p0, p1, z_ground, eps=1e-9):
    z0, z1 = p0[2], p1[2]
    dz = z1 - z0
    if abs(dz) < eps:
        return None, None
    t = (z_ground - z0) / dz
    if -eps <= t <= 1.0 + eps:
        t = max(0.0, min(1.0, float(t)))
        hit = p0 + t * (p1 - p0)
        return hit, t
    return None, None

def first_contact_with_t(mesh, p0, p1, z_ground):
    """p0->p1でSTL/地面のうち先に当たるもの．戻り：(hit(3,), type['stl'|'ground'], t)"""
    pts, _ = mesh.ray_trace(p0, p1)
    hit_stl, t_stl = None, None
    if pts is not None and len(pts) > 0:
        hit_stl = np.array(pts[0], dtype=float)
        seg_len = np.linalg.norm(p1 - p0)
        if seg_len > 1e-9:
            t_stl = np.linalg.norm(hit_stl - p0) / seg_len

    hit_g, t_g = segment_ground_intersection(p0, p1, z_ground)

    if t_stl is None and t_g is None:
        return None, None, None
    if t_stl is None:
        return np.array(hit_g), "ground", t_g
    if t_g is None:
        return hit_stl, "stl", t_stl
    if t_g <= t_stl:
        return np.array(hit_g), "ground", t_g
    else:
        return hit_stl, "stl", t_stl

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

# ---------- 解析ユーティリティ ----------
def count_ids_and_max_points(plan_json_path):
    """①id数，②理論上の最大点数（Pb総数）"""
    with open(plan_json_path, "r", encoding="utf-8") as f:
        data = json.load(f)
    ids = data.get("plan", [])
    id_count = len(ids)
    pb_total = 0
    for blk in ids:
        Pb = blk.get("Pb", [])
        pb_total += len(Pb)
    return id_count, pb_total

# ---------- シミュレーション ----------
def simulate_contacts_ideal(mesh, plan_json_path, z_ground):
    with open(plan_json_path, "r", encoding="utf-8") as f:
        data = json.load(f)

    contacts = []
    for blk in data.get("plan", []):
        pid = blk.get("id")
        Pa, Pb = blk.get("Pa", []), blk.get("Pb", [])
        n = min(len(Pa), len(Pb))
        for i in range(n):
            p0 = np.array(Pa[i], dtype=float)
            p1 = np.array(Pb[i], dtype=float)
            hit, typ, _t = first_contact_with_t(mesh, p0, p1, z_ground)
            if typ is not None:
                contacts.append({"id": pid, "pin_index": i, "type": typ, "point": list(map(float, hit))})
    return contacts, []  # ×はなし

def simulate_contacts_realistic(mesh, plan_json_path, z_ground, pin_len_mm,
                                mark_x_for_nohit=True, collect_ideal_unreached=True):
    """
    現実：全ピンの理想接触tを求め，最小t=t_firstから pin_len_mm だけ押し込み可 → t_stop
    0..t_stopの範囲での接触を採用．届かないピンはp_stopに×．
    ideal_unreached（赤点）は「届かないピン」の理想接触点（STL/地面の双方）→ 必ず×と1:1に対応
    """
    with open(plan_json_path, "r", encoding="utf-8") as f:
        data = json.load(f)

    contacts, xmarks, ideal_unreached = [], [], []

    for blk in data.get("plan", []):
        pid = blk.get("id")
        Pa, Pb = blk.get("Pa", []), blk.get("Pb", [])
        n = min(len(Pa), len(Pb))
        if n == 0:
            continue

        # 理想（0..1）の最初の接触
        ideal_hits = []  # [{type, point, t}]
        t_first = None
        for i in range(n):
            p0 = np.array(Pa[i], dtype=float); p1 = np.array(Pb[i], dtype=float)
            hit, typ, t = first_contact_with_t(mesh, p0, p1, z_ground)
            ideal_hits.append({"type": typ, "point": hit, "t": t})
            if t is not None:
                t_first = t if (t_first is None or t < t_first) else t_first

        # t_add（t単位換算）
        p0_ref = np.array(Pa[0], dtype=float); p1_ref = np.array(Pb[0], dtype=float)
        dz_total = abs(p1_ref[2] - p0_ref[2])
        if dz_total > 1e-9:
            t_add = float(pin_len_mm) / dz_total
        else:
            seg_len = np.linalg.norm(p1_ref - p0_ref)
            t_add = (float(pin_len_mm) / seg_len) if seg_len > 1e-9 else 0.0
        t_stop = 1.0 if t_first is None else min(1.0, t_first + t_add)

        # 0..t_stopでの接触
        real_types = [None]*n
        for i in range(n):
            p0 = np.array(Pa[i], dtype=float); p1 = np.array(Pb[i], dtype=float)
            p_stop = p0 + t_stop * (p1 - p0)
            hit, typ, _t = first_contact_with_t(mesh, p0, p_stop, z_ground)
            if typ is not None:
                contacts.append({"id": pid, "pin_index": i, "type": typ, "point": list(map(float, hit))})
                real_types[i] = typ
            else:
                if mark_x_for_nohit:
                    xmarks.append({"id": pid, "pin_index": i, "point": list(map(float, p_stop))})
                real_types[i] = None

        # 届かないピンの理想接触点（STL/地面の双方）→ 赤点
        if collect_ideal_unreached:
            for i in range(n):
                ih = ideal_hits[i]
                if ih["t"] is not None:  # 理想では何かに触れている
                    if real_types[i] is None:  # 現実では届いていない
                        ideal_unreached.append({
                            "id": pid, "pin_index": i,
                            "type": ih["type"],  # 'stl' or 'ground'
                            "point": list(map(float, ih["point"]))
                        })

    return contacts, xmarks, ideal_unreached

# ---------- GUI ----------
class MainWindow(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("接触点生成GUI（点群のみ表示／2モード＋赤点＆統計）")
        self.resize(1250, 900)

        default_stl  = os.path.join(BASE, "pumpkin1.stl")
        default_json = os.path.join(BASE, "push_plan.json")

        # ファイル
        self.stl_label = QtWidgets.QLabel("STLファイル:")
        self.stl_path  = QtWidgets.QLineEdit(default_stl if os.path.exists(default_stl) else "pumpkin1.stl")
        self.stl_browse = QtWidgets.QPushButton("参照")
        self.json_label = QtWidgets.QLabel("計画JSON:")
        self.json_path  = QtWidgets.QLineEdit(default_json if os.path.exists(default_json) else "push_plan.json")
        self.json_browse = QtWidgets.QPushButton("参照")

        # デシメ
        self.decim_check = QtWidgets.QCheckBox("簡略化を使う（デシメーション）"); self.decim_check.setChecked(True)
        self.decim_label = QtWidgets.QLabel("削減率（%）:")
        self.decim_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal); self.decim_slider.setRange(0, 90); self.decim_slider.setValue(60)
        self.decim_spin   = QtWidgets.QSpinBox(); self.decim_spin.setRange(0, 90); self.decim_spin.setValue(60)
        self.decim_slider.valueChanged.connect(self.decim_spin.setValue)
        self.decim_spin.valueChanged.connect(self.decim_slider.setValue)

        # 地面Z
        self.ground_label = QtWidgets.QLabel("地面Z [mm]:")
        self.ground_edit  = QtWidgets.QLineEdit("0")

        # モード
        self.mode_label = QtWidgets.QLabel("シミュレーション方式:")
        self.mode_combo = QtWidgets.QComboBox()
        self.mode_combo.addItems(["理想（無制限）", "現実（ピン長制約）"])

        # ピン長
        self.pinlen_label = QtWidgets.QLabel("ピン長 [mm]（現実モード）:")
        self.pinlen_edit  = QtWidgets.QLineEdit("30")

        # ×マーク
        self.xmark_check = QtWidgets.QCheckBox("未接触ピン先端に×を描画（現実モード）"); self.xmark_check.setChecked(True)
        # 赤点（届かないピンの理想接触点：STL/地面ともに）
        self.redideal_check = QtWidgets.QCheckBox("届かないピンの理想接触点を赤点で表示（STL/地面）"); self.redideal_check.setChecked(True)

        # 実行
        self.generate_btn = QtWidgets.QPushButton("生成（計算＆表示）")
        self.save_simplified_btn = QtWidgets.QPushButton("簡略化STLを保存")

        # Plotter（点群のみ）
        self.plotter = QtInteractor(self); self.plotter.set_background("white")

        # 統計表示
        stats_box = QtWidgets.QGroupBox("統計")
        self.stat_1 = QtWidgets.QLabel("① 押し込み回数(id)：-")
        self.stat_2 = QtWidgets.QLabel("② 理論最大点数(Pb総数)：-")
        self.stat_3 = QtWidgets.QLabel("③ 実接触点数(青)：-")
        self.stat_4 = QtWidgets.QLabel("④ ③/② [％]：-")
        self.stat_5 = QtWidgets.QLabel("⑤ 青のうちSTL接触数：-")
        self.stat_6 = QtWidgets.QLabel("⑥ ⑤/② [％]：-")
        self.stat_7 = QtWidgets.QLabel("⑦ 未接触(×)数：-")
        vstats = QtWidgets.QVBoxLayout(stats_box)
        for w in [self.stat_1,self.stat_2,self.stat_3,self.stat_4,self.stat_5,self.stat_6,self.stat_7]:
            vstats.addWidget(w)

        # レイアウト
        form = QtWidgets.QGridLayout(); row = 0
        form.addWidget(self.stl_label, row,0); form.addWidget(self.stl_path,row,1); form.addWidget(self.stl_browse,row,2); row+=1
        form.addWidget(self.json_label,row,0); form.addWidget(self.json_path,row,1); form.addWidget(self.json_browse,row,2); row+=1
        form.addWidget(self.decim_check,row,0); row+=1
        form.addWidget(self.decim_label,row,0); form.addWidget(self.decim_slider,row,1); form.addWidget(self.decim_spin,row,2); row+=1
        form.addWidget(self.ground_label,row,0); form.addWidget(self.ground_edit,row,1); row+=1
        form.addWidget(self.mode_label,row,0); form.addWidget(self.mode_combo,row,1); row+=1
        form.addWidget(self.pinlen_label,row,0); form.addWidget(self.pinlen_edit,row,1); row+=1
        form.addWidget(self.xmark_check,row,0,1,3); row+=1
        form.addWidget(self.redideal_check,row,0,1,3); row+=1
        form.addWidget(self.generate_btn,row,0); form.addWidget(self.save_simplified_btn,row,1,1,2); row+=1

        layout = QtWidgets.QVBoxLayout(self)
        layout.addLayout(form)
        layout.addWidget(self.plotter.interactor)
        layout.addWidget(stats_box)

        # 状態
        self.mesh = None
        self.contacts = []        # 青点
        self.xmarks = []          # ×
        self.ideal_unreached = [] # 赤点
        self.id_count = 0
        self.pb_total = 0

        # シグナル
        self.stl_browse.clicked.connect(self.on_browse_stl)
        self.json_browse.clicked.connect(self.on_browse_json)
        self.generate_btn.clicked.connect(self.on_generate)
        self.save_simplified_btn.clicked.connect(self.on_save_simplified)

    # ------ UI ------
    def on_browse_stl(self):
        path, _ = QtWidgets.QFileDialog.getOpenFileName(self, "Select STL", BASE, "STL (*.stl)")
        if path: self.stl_path.setText(os.path.relpath(path, BASE))
    def on_browse_json(self):
        path, _ = QtWidgets.QFileDialog.getOpenFileName(self, "Select JSON", BASE, "JSON (*.json)")
        if path: self.json_path.setText(os.path.relpath(path, BASE))

    # ------ 実行 ------
    def on_generate(self):
        try:
            stl_abs  = abspath_from_base(self.stl_path.text())
            json_abs = abspath_from_base(self.json_path.text())
            z_ground = float(self.ground_edit.text().strip())
            use_decim = self.decim_check.isChecked()
            reduction = self.decim_spin.value() / 100.0
            mode = self.mode_combo.currentIndex()

            if not os.path.exists(stl_abs):
                QtWidgets.QMessageBox.warning(self, "ファイルなし", f"STLが見つかりません:\n{stl_abs}"); return
            if not os.path.exists(json_abs):
                QtWidgets.QMessageBox.warning(self, "ファイルなし", f"JSONが見つかりません:\n{json_abs}"); return

            # 統計の①②
            self.id_count, self.pb_total = count_ids_and_max_points(json_abs)

            # メッシュ
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

            # シミュレーション
            self.ideal_unreached = []
            if mode == 0:
                self.contacts, self.xmarks = simulate_contacts_ideal(self.mesh, json_abs, z_ground)
            else:
                try:
                    pin_len = float(self.pinlen_edit.text().strip())
                except:
                    QtWidgets.QMessageBox.warning(self, "入力エラー", "ピン長は数値で入力してください"); return
                self.contacts, self.xmarks, self.ideal_unreached = simulate_contacts_realistic(
                    self.mesh, json_abs, z_ground, pin_len_mm=pin_len,
                    mark_x_for_nohit=self.xmark_check.isChecked(),
                    collect_ideal_unreached=True
                )

            # 描画
            self.refresh_plot(mode)

            # 統計更新（③〜⑦）
            self.update_stats()

            # 保存
            out_path = os.path.join(BASE, "contact_points.json")
            payload = {
                "ground_z": z_ground,
                "source": {"stl": stl_abs, "push_plan": json_abs, "decimated": use_decim, "target_reduction": reduction},
                "mode": "ideal" if mode == 0 else "realistic",
                "pin_length_mm": float(self.pinlen_edit.text().strip()) if mode == 1 else None,
                "contacts": self.contacts,
                "xmarks": self.xmarks if self.xmark_check.isChecked() and mode == 1 else [],
                "ideal_unreached": self.ideal_unreached if self.redideal_check.isChecked() and mode == 1 else []
            }
            with open(out_path, "w", encoding="utf-8") as f:
                json.dump(payload, f, ensure_ascii=False, indent=2)
            log.info(f"結果を保存: {out_path}")
            QtWidgets.QMessageBox.information(self, "保存完了", f"結果を保存しました:\n{out_path}")

        except Exception as e:
            log.error(f"計算失敗: {e}")
            log.error(traceback.format_exc())
            QtWidgets.QMessageBox.critical(self, "計算失敗", str(e))

    # ------ 可視化（点群のみ） ------
    def refresh_plot(self, mode_index):
        self.plotter.clear()

        # 青点（実際に接触した点）: 濃い青
        if self.contacts:
            pts = np.array([c["point"] for c in self.contacts], dtype=float)
            cloud = pv.PolyData(pts)
            self.plotter.add_mesh(cloud, render_points_as_spheres=True, point_size=12, color="blue")

        # ×マーク（現実モードかつON）
        if mode_index == 1 and self.xmark_check.isChecked():
            for xm in self.xmarks:
                self.add_x_marker(np.array(xm["point"], dtype=float), size=6.0, lw=2)

        # 赤点（届かないピンの理想接触点：STL/地面）: 現実モードかつON
        if mode_index == 1 and self.redideal_check.isChecked() and self.ideal_unreached:
            red_pts = np.array([d["point"] for d in self.ideal_unreached], dtype=float)
            red_cloud = pv.PolyData(red_pts)
            self.plotter.add_mesh(red_cloud, render_points_as_spheres=True, point_size=12, color="red")

        self.plotter.reset_camera()

    def add_x_marker(self, center, size=5.0, lw=2):
        d = size * 0.5
        p1 = center + np.array([ d,  d, 0.0])
        p2 = center + np.array([-d, -d, 0.0])
        p3 = center + np.array([ d, -d, 0.0])
        p4 = center + np.array([-d,  d, 0.0])
        self.plotter.add_mesh(pv.Line(p1, p2), line_width=lw)
        self.plotter.add_mesh(pv.Line(p3, p4), line_width=lw)

    # ------ 統計更新 ------
    def update_stats(self):
        contact_total = len(self.contacts)                # ③
        stl_contacts = sum(1 for c in self.contacts if c["type"] == "stl")  # ⑤
        x_total = len(self.xmarks)                        # ⑦
        denom = max(self.pb_total, 1)
        ratio_all = 100.0 * contact_total / denom        # ④
        ratio_stl = 100.0 * stl_contacts / denom         # ⑥

        self.stat_1.setText(f"① 押し込み回数(id)：{self.id_count}")
        self.stat_2.setText(f"② 理論最大点数(Pb総数)：{self.pb_total}")
        self.stat_3.setText(f"③ 実接触点数(青)：{contact_total}")
        self.stat_4.setText(f"④ ③/② [％]：{ratio_all:.2f}")
        self.stat_5.setText(f"⑤ 青のうちSTL接触数：{stl_contacts}")
        self.stat_6.setText(f"⑥ ⑤/② [％]：{ratio_stl:.2f}")
        self.stat_7.setText(f"⑦ 未接触(×)数：{x_total}")

    # ------ 簡略化STL保存 ------
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
