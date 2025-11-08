# -*- coding: utf-8 -*-
# viewer_gui.py (拡張版)
# 機能追加:
#  - STLファイル選択ダイアログ
#  - push_plan.json選択ダイアログ
#  - push_plan.jsonのPa→PbをもとにSTLとの接触点をレイキャストでシミュレーション
#  - 接触シミュレーション開始ボタン
#  - シミュレーション結果表示ON/OFF
#  - STL表示ON/OFF
#  - 高さ(Z)を2色グラデーションで表示（低:明るい緑 → 高:濃い緑）

import os
import sys
import json
import pyvista as pv
import numpy as np
from pyvistaqt import QtInteractor
from PyQt5 import QtWidgets, QtCore, QtGui
from matplotlib.colors import LinearSegmentedColormap

# ===== ユーザ設定 =====
STL_NAME = "pumpkin_down_on_desk(170_200_150)(500-0-30_1-180-0-90).stl"     # デフォルトのSTL
FACE_THRESHOLD = 100_000               # この面数を超えたらデシメーション
TARGET_REDUCTION = 0.90                # デシメーション削減率(0.90=90%削減)
WINDOW_SIZE = (1200, 800)
# =====================


def make_green_cmap():
    """
    高さを2色(明→暗)で連続的に変化させるカラーマップを作る．
    0.0: 明るい緑, 1.0: 暗い緑
    """
    low = (0xA8/255, 0xE6/255, 0xA1/255)   # #a8e6a1 (低いところ)
    high = (0x00/255, 0x42/255, 0x25/255)  # #004225 (高いところ)
    cmap = LinearSegmentedColormap.from_list(
        "green_height",
        [low, high],
        N=256,
    )
    return cmap


class STLViewer(QtWidgets.QMainWindow):
    def __init__(self, parent=None):
        super().__init__(parent)

        self.setWindowTitle("STL Viewer (PyVista + Qt)")
        self.resize(*WINDOW_SIZE)

        # --- 状態管理 ---
        self.current_dir = os.path.dirname(os.path.abspath(__file__))
        self.stl_path = os.path.join(self.current_dir, STL_NAME)
        self.push_plan_path = None

        self.mesh = None
        self.mesh_actor = None
        self.bbox_actor = None
        self.center_actor = None
        self.scene_text_actor = None
        self.highlight_actor = None

        # plan関連
        self.plan_entries = []   # ← idごとに辞書で持つ
        self.selected_ids = set()

        # 以前のグローバル配列も一応残す
        self.contact_actors = []
        self.pa_actors = []
        self.miss_actors = []

        # 現実モード関連
        self.sim_realistic = False
        self.pin_length_mm = 30.0

        # 中央レイアウト
        central = QtWidgets.QWidget(self)
        self.setCentralWidget(central)
        layout = QtWidgets.QHBoxLayout(central)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(8)

        # 左: 3Dビュー
        self.plotter = QtInteractor(self)
        layout.addWidget(self.plotter, stretch=1)

        # 右: コントロールパネル
        side = QtWidgets.QFrame(self)
        side.setFrameShape(QtWidgets.QFrame.StyledPanel)
        side_layout = QtWidgets.QVBoxLayout(side)
        side_layout.setContentsMargins(8, 8, 8, 8)
        side_layout.setSpacing(10)
        layout.addWidget(side, stretch=0)

        # --- ここから右ペインの部品 ---

        # STLファイル表示・選択
        self.lbl_stl = QtWidgets.QLabel(f"STL: {os.path.basename(self.stl_path)}", self)
        side_layout.addWidget(self.lbl_stl)
        self.btn_stl = QtWidgets.QPushButton("STLを選択...", self)
        self.btn_stl.clicked.connect(self.on_select_stl)
        side_layout.addWidget(self.btn_stl)

        # push_plan表示・選択
        self.lbl_push = QtWidgets.QLabel("push_plan: (未選択)", self)
        side_layout.addWidget(self.lbl_push)
        self.btn_push = QtWidgets.QPushButton("push_plan.jsonを選択...", self)
        self.btn_push.clicked.connect(self.on_select_push_plan)
        side_layout.addWidget(self.btn_push)

        # 視点リセット
        self.btn_reset = QtWidgets.QPushButton("視点リセット (ZY正面 / XY水平)", self)
        self.btn_reset.clicked.connect(self.reset_view)
        side_layout.addWidget(self.btn_reset)

        # バウンディングボックス
        self.chk_bbox = QtWidgets.QCheckBox("バウンディングボックスを表示", self)
        self.chk_bbox.stateChanged.connect(self.on_bbox_toggled)
        side_layout.addWidget(self.chk_bbox)

        # STL表示ON/OFF
        self.chk_show_stl = QtWidgets.QCheckBox("STLを表示", self)
        self.chk_show_stl.setChecked(True)
        self.chk_show_stl.stateChanged.connect(self.on_show_stl_toggled)
        side_layout.addWidget(self.chk_show_stl)

        # シミュレーション表示
        self.chk_show_sim = QtWidgets.QCheckBox("シミュレーション結果を表示", self)
        self.chk_show_sim.setChecked(True)
        self.chk_show_sim.stateChanged.connect(self.on_show_sim_toggled)
        side_layout.addWidget(self.chk_show_sim)

        # Pa表示
        self.chk_show_pa = QtWidgets.QCheckBox("ピン押下スタート(Pa)を表示", self)
        self.chk_show_pa.setChecked(True)
        self.chk_show_pa.stateChanged.connect(self.on_show_pa_toggled)
        side_layout.addWidget(self.chk_show_pa)

        # 未接触×表示
        self.chk_show_miss = QtWidgets.QCheckBox("未接触ピンの終端(Pb)を表示", self)
        self.chk_show_miss.setChecked(True)
        self.chk_show_miss.stateChanged.connect(self.on_show_miss_toggled)
        side_layout.addWidget(self.chk_show_miss)

        # 現実モード
        self.chk_real = QtWidgets.QCheckBox("現実モード(ピン長さ制限)", self)
        self.chk_real.setChecked(False)
        self.chk_real.stateChanged.connect(self.on_real_mode_toggled)
        side_layout.addWidget(self.chk_real)

        # ピン長さ
        hbox_pin = QtWidgets.QHBoxLayout()
        hbox_pin.addWidget(QtWidgets.QLabel("ピン長さ[mm]:", self))
        self.spin_pinlen = QtWidgets.QDoubleSpinBox(self)
        self.spin_pinlen.setRange(1.0, 500.0)
        self.spin_pinlen.setValue(30.0)
        self.spin_pinlen.setSingleStep(1.0)
        self.spin_pinlen.valueChanged.connect(self.on_pinlen_changed)
        hbox_pin.addWidget(self.spin_pinlen)
        side_layout.addLayout(hbox_pin)

        # シミュレーション開始
        self.btn_sim = QtWidgets.QPushButton("接触シミュレーション開始", self)
        self.btn_sim.clicked.connect(self.on_run_simulation)
        side_layout.addWidget(self.btn_sim)

        # ★ id選択モード
        self.chk_multi_id = QtWidgets.QCheckBox("idを複数選択する", self)
        self.chk_multi_id.setChecked(True)   # ← 初期値は複数選択ON
        side_layout.addWidget(self.chk_multi_id)

        # ★ id一覧リスト
        self.id_list = QtWidgets.QListWidget(self)
        self.id_list.itemClicked.connect(self.on_id_item_clicked)
        self.id_list.setMinimumHeight(120)
        side_layout.addWidget(QtWidgets.QLabel("ID一覧:", self))
        side_layout.addWidget(self.id_list)


        # ★ 選択クリア
        self.btn_clear_sel = QtWidgets.QPushButton("選択をクリア", self)
        self.btn_clear_sel.clicked.connect(self.on_clear_selection)
        side_layout.addWidget(self.btn_clear_sel)

        # ★ 情報表示パネル
        info_group = QtWidgets.QGroupBox("選択IDの情報", self)
        info_layout = QtWidgets.QVBoxLayout(info_group)

        self.chk_info_A = QtWidgets.QCheckBox("Aの座標", self)
        self.chk_info_A.setChecked(True)
        self.chk_info_B = QtWidgets.QCheckBox("Bの座標", self)
        self.chk_info_B.setChecked(True)
        self.chk_info_Pa = QtWidgets.QCheckBox("Paの座標群", self)
        self.chk_info_Pa.setChecked(False)
        self.chk_info_Pb = QtWidgets.QCheckBox("Pbの座標群", self)
        self.chk_info_Pb.setChecked(False)
        self.chk_info_plot = QtWidgets.QCheckBox("各ピンのプロット座標", self)
        self.chk_info_plot.setChecked(True)

        # チェックを変えたら表示を更新
        for w in (self.chk_info_A, self.chk_info_B, self.chk_info_Pa,
                  self.chk_info_Pb, self.chk_info_plot):
            w.stateChanged.connect(self.update_selected_info)

        info_layout.addWidget(self.chk_info_A)
        info_layout.addWidget(self.chk_info_B)
        info_layout.addWidget(self.chk_info_Pa)
        info_layout.addWidget(self.chk_info_Pb)
        info_layout.addWidget(self.chk_info_plot)

        self.info_text = QtWidgets.QTextEdit(self)
        self.info_text.setReadOnly(True)
        self.info_text.setMinimumHeight(180)
        info_layout.addWidget(self.info_text)

        side_layout.addWidget(info_group)

        # 下の寸法表示はとりあえず残す
        self.dim_text = QtWidgets.QTextEdit(self)
        self.dim_text.setReadOnly(True)
        self.dim_text.setFixedHeight(120)
        side_layout.addWidget(self.dim_text)

        side_layout.addStretch(1)

        # 初期読み込み
        self.load_mesh_and_setup(self.stl_path)

    # ==============================
    # ファイル選択まわり
    # ==============================
    def on_select_stl(self):
        path, _ = QtWidgets.QFileDialog.getOpenFileName(
            self,
            "STLを選択",
            self.current_dir,
            "STL Files (*.stl);;All Files (*)"
        )
        if not path:
            return
        self.stl_path = path
        self.lbl_stl.setText(f"STL: {os.path.basename(path)}")
        self.load_mesh_and_setup(path)

    def on_select_push_plan(self):
        path, _ = QtWidgets.QFileDialog.getOpenFileName(
            self,
            "push_plan.jsonを選択",
            self.current_dir,
            "JSON Files (*.json);;All Files (*)"
        )
        if not path:
            return
        self.push_plan_path = path
        self.lbl_push.setText(f"push_plan: {os.path.basename(path)}")
        self.load_push_plan(path)

    # ==============================
    # push_planの読み込みとパース
    # ==============================
    def load_push_plan(self, json_path: str):
        try:
            with open(json_path, "r", encoding="utf-8") as f:
                data = json.load(f)
        except Exception as e:
            QtWidgets.QMessageBox.critical(self, "エラー", f"push_planの読み込みに失敗しました:\n{e}")
            return

        self.plan_entries = []
        self.id_list.clear()

        plans = data.get("plan", [])
        for item in plans:
            pid = item.get("id")
            A = item.get("A", [])
            B = item.get("B", [])
            Pa_list = item.get("Pa", [])
            Pb_list = item.get("Pb", [])

            segments = []
            n = min(len(Pa_list), len(Pb_list))
            for i in range(n):
                pa = Pa_list[i]
                pb = Pb_list[i]
                if len(pa) >= 3 and len(pb) >= 3:
                    segments.append((pa, pb))

            entry = {
                "id": pid,
                "A": A,
                "B": B,
                "Pa_list": Pa_list,
                "Pb_list": Pb_list,
                "segments": segments,
                # 描画済みactorをここに入れていく
                "actors": {
                    "pa": [],
                    "contact": [],
                    "miss": [],
                },
                # シミュレーションした結果(各ピンごと)
                "last_results": [],  # {idx, status:'hit'/'miss', point:(x,y,z)}
            }
            self.plan_entries.append(entry)

            # GUIのリストにも追加
            item_widget = QtWidgets.QListWidgetItem(f"id: {pid}")
            # 選択色を後でつけやすいようにしておく
            self.id_list.addItem(item_widget)

        self.dim_text.append(f"[push_plan] 読み込み: {len(self.plan_entries)}個のid")

        # Paを描画
        self.draw_pa_points()

    def on_real_mode_toggled(self, state):
        self.sim_realistic = (state == QtCore.Qt.Checked)
        self.dim_text.append(f"[mode] 現実モード = {self.sim_realistic}")

    def on_pinlen_changed(self, val: float):
        self.pin_length_mm = float(val)
        self.dim_text.append(f"[param] ピン長さ = {self.pin_length_mm:.1f} mm")


    # ==============================
    # メッシュ読み込みと描画初期化
    # ==============================
    def load_mesh_and_setup(self, stl_path: str):
        if not os.path.exists(stl_path):
            QtWidgets.QMessageBox.critical(self, "エラー", f"STLが見つからない: {stl_path}")
            return

        mesh = pv.read(stl_path)
        n_faces = mesh.n_cells
        n_pts = mesh.n_points

        decimated = False
        if n_faces > FACE_THRESHOLD:
            mesh = mesh.decimate(target_reduction=TARGET_REDUCTION)
            decimated = True

        z = mesh.points[:, 2]
        mesh["z_height"] = z
        zmin = float(z.min())
        zmax = float(z.max())
        if zmax == zmin:
            zmax = zmin + 1e-6

        green_cmap = make_green_cmap()

        self.mesh = mesh

        self.plotter.clear()
        self.remove_bbox_actors()
        self.clear_contact_actors()
        self.clear_pa_actors()
        self.clear_miss_actors()

        self.mesh_actor = self.plotter.add_mesh(
            self.mesh,
            scalars="z_height",
            cmap=green_cmap,
            show_edges=False,
            smooth_shading=True,
            clim=(zmin, zmax),
            nan_color=(0, 0.25, 0.15),
            above_color=(0, 0.25, 0.15),
            below_color=(0.66, 0.9, 0.63),
            n_colors=256,
            scalar_bar_args={"title": "Height (Z)"},
        )
        self.mesh_actor.SetVisibility(self.chk_show_stl.isChecked())

        self.plotter.add_axes()
        self.plotter.show_grid()
        self.reset_view()

    # ==============================
    # 接触シミュレーション
    # ==============================
    def on_run_simulation(self):
        if self.mesh is None:
            QtWidgets.QMessageBox.warning(self, "警告", "先にSTLを読み込んでください．")
            return
        if not self.plan_entries:
            QtWidgets.QMessageBox.warning(self, "警告", "先にpush_plan.jsonを読み込んでください．")
            return

        # クリア
        self.clear_contact_actors()
        self.clear_miss_actors()
        # 各entryのactorsもクリア
        for entry in self.plan_entries:
            entry["actors"]["contact"] = []
            entry["actors"]["miss"] = []
            entry["last_results"] = []

        if self.sim_realistic:
            self._run_simulation_realistic()
        else:
            self._run_simulation_ideal()

        # シミュレーションしたので，選択されていたidの色をもう一回反映
        self._refresh_selected_colors()
        self._refresh_id_list_colors()
        # 最後に選ばれていたものをもう一度囲む
        if self.selected_ids:
            # setなので順番はないが，最後に選んだidをどこかで保持しているならそれを使う
            # ここではとりあえず最大idを囲む
            last_id = list(self.selected_ids)[-1]
            self._highlight_entry_bbox(last_id)
        self.update_selected_info()
            
    def _run_simulation_ideal(self):
        radius = self._contact_radius_from_mesh()
        miss_size = radius * 2.0

        total_hit = 0
        total_miss = 0

        for entry in self.plan_entries:
            for idx, (pa, pb) in enumerate(entry["segments"]):
                pa_xyz = np.asarray(pa, dtype=float)
                pb_xyz = np.asarray(pb, dtype=float)
                points, ind = self.mesh.ray_trace(pa_xyz, pb_xyz)
                if points.size > 0:
                    pt = points[0]
                    sphere = pv.Sphere(radius=radius, center=pt)
                    actor = self.plotter.add_mesh(sphere, color="red")
                    actor.SetVisibility(self.chk_show_sim.isChecked())
                    self.contact_actors.append(actor)
                    entry["actors"]["contact"].append(actor)
                    entry["last_results"].append({
                        "pin": idx + 1,
                        "status": "〇",
                        "point": tuple(pt),
                    })
                    total_hit += 1
                else:
                    actors = self._add_cross_at(tuple(pb_xyz), miss_size)
                    for a in actors:
                        self.miss_actors.append(a)
                        entry["actors"]["miss"].append(a)
                    entry["last_results"].append({
                        "pin": idx + 1,
                        "status": "×",
                        "point": tuple(pb_xyz),
                    })
                    total_miss += 1

        self.plotter.render()
        self.dim_text.append(f"[sim ideal] 接触 {total_hit}，未接触 {total_miss}")

    def _run_simulation_realistic(self):
        radius = self._contact_radius_from_mesh()
        miss_size = radius * 3.0
        pin_len = float(self.pin_length_mm)

        total_hit = 0
        total_miss = 0

        for entry in self.plan_entries:
            segs = entry["segments"]
            if not segs:
                continue

            per_pin = []
            d_min = None

            # 1) まず各ピンの「当たりまでの距離」を調べる
            for idx, (pa, pb) in enumerate(segs):
                pa_xyz = np.asarray(pa, dtype=float)
                pb_xyz = np.asarray(pb, dtype=float)
                vec = pb_xyz - pa_xyz
                planned = float(np.linalg.norm(vec))
                if planned < 1e-6:
                    per_pin.append({
                        "pin": idx + 1,
                        "pa": pa_xyz,
                        "pb": pb_xyz,
                        "planned": planned,
                        "dir": None,
                        "hit": False,
                        "hit_dist": None,
                        "hit_point": None,
                    })
                    continue

                dir_unit = vec / planned
                points, ind = self.mesh.ray_trace(pa_xyz, pb_xyz)
                if points.size > 0:
                    hit_pt = points[0]
                    hit_dist = float(np.linalg.norm(hit_pt - pa_xyz))
                    hit_flag = True
                    if d_min is None or hit_dist < d_min:
                        d_min = hit_dist
                else:
                    hit_pt = None
                    hit_dist = None
                    hit_flag = False

                per_pin.append({
                    "pin": idx + 1,
                    "pa": pa_xyz,
                    "pb": pb_xyz,
                    "planned": planned,
                    "dir": dir_unit,
                    "hit": hit_flag,
                    "hit_dist": hit_dist,
                    "hit_point": hit_pt,
                })

            # 2) グループ全体で実際に押し込める量
            planned_max = max(p["planned"] for p in per_pin)
            if d_min is None:
                actual_travel = planned_max
            else:
                actual_travel = min(planned_max, d_min + pin_len)

            # 3) 各ピンの最終位置を描画
            for p in per_pin:
                if p["hit"] and actual_travel >= p["hit_dist"]:
                    sphere = pv.Sphere(radius=radius, center=p["hit_point"])
                    actor = self.plotter.add_mesh(sphere, color="red")
                    actor.SetVisibility(self.chk_show_sim.isChecked())
                    self.contact_actors.append(actor)
                    entry["actors"]["contact"].append(actor)
                    entry["last_results"].append({
                        "pin": p["pin"],
                        "status": "〇",
                        "point": tuple(p["hit_point"]),
                    })
                    total_hit += 1
                else:
                    # 実際にここまでしか行けない
                    if p["dir"] is None or actual_travel < 1e-6:
                        final_pt = p["pa"]
                    else:
                        step = min(actual_travel, p["planned"])
                        final_pt = p["pa"] + p["dir"] * step

                    actors = self._add_cross_at(tuple(final_pt), miss_size)
                    for a in actors:
                        self.miss_actors.append(a)
                        entry["actors"]["miss"].append(a)
                    entry["last_results"].append({
                        "pin": p["pin"],
                        "status": "×",
                        "point": tuple(final_pt),
                    })
                    total_miss += 1

        self.plotter.render()
        self.dim_text.append(f"[sim real] 接触 {total_hit}，未接触 {total_miss} (pin={self.pin_length_mm:.1f}mm)")


    def _contact_radius_from_mesh(self) -> float:
        return 5.0  # mm想定

    def clear_contact_actors(self):
        """既存の赤丸を全部消す"""
        if not self.contact_actors:
            return
        for actor in self.contact_actors:
            try:
                self.plotter.remove_actor(actor)
            except Exception:
                pass
        self.contact_actors = []

    def draw_pa_points(self):
        self.clear_pa_actors()
        if not self.plan_entries or self.mesh is None:
            return

        radius = self._contact_radius_from_mesh()

        for entry in self.plan_entries:
            entry["actors"]["pa"] = []
            for pa in entry["Pa_list"]:
                center = (pa[0], pa[1], pa[2])
                sphere = pv.Sphere(radius=radius, center=center)
                actor = self.plotter.add_mesh(
                    sphere,
                    color="red",
                    style="wireframe",
                    line_width=2.0,
                )
                actor.SetVisibility(self.chk_show_pa.isChecked())
                entry["actors"]["pa"].append(actor)
                self.pa_actors.append(actor)

        self.plotter.render()


    def clear_pa_actors(self):
        """Pa用の赤丸を全部消す"""
        if not self.pa_actors:
            return
        for actor in self.pa_actors:
            try:
                self.plotter.remove_actor(actor)
            except Exception:
                pass
        self.pa_actors = []
    
    def _add_cross_at(self, center, size):
        """
        center: (x,y,z)
        size: crossの全長
        """
        x, y, z = center
        half = size / 2.0

        line1 = pv.Line((x - half, y - half, z), (x + half, y + half, z))
        line2 = pv.Line((x - half, y + half, z), (x + half, y - half, z))

        actor1 = self.plotter.add_mesh(line1, color="red", line_width=5.0)
        actor2 = self.plotter.add_mesh(line2, color="red", line_width=5.0)

        actor1.SetVisibility(self.chk_show_miss.isChecked())
        actor2.SetVisibility(self.chk_show_miss.isChecked())

        return [actor1, actor2]


    def clear_miss_actors(self):
        """非接触ピンの×を全部消す"""
        if not self.miss_actors:
            return
        for actor in self.miss_actors:
            try:
                self.plotter.remove_actor(actor)
            except Exception:
                pass
        self.miss_actors = []

    def _highlight_entry_bbox(self, pid: int):
        """指定idのプロットを黄色い直方体で囲む"""
        # まず以前のハイライトを消す
        self._clear_highlight_bbox()

        # エントリを探す
        target = None
        for entry in self.plan_entries:
            if entry["id"] == pid:
                target = entry
                break
        if target is None:
            return

        # そのidで可視化されているポイントを全部集める
        pts = []

        # Pa
        for pa in target["Pa_list"]:
            pts.append(pa)

        # シミュレーション済みのプロット
        for res in target["last_results"]:
            pts.append(res["point"])

        if not pts:
            return

        pts = np.asarray(pts, dtype=float)
        xmin, ymin, zmin = pts.min(axis=0)
        xmax, ymax, zmax = pts.max(axis=0)

        # 少し余裕を持たせる
        margin = 5.0  # mm
        bounds = (
            xmin - margin, xmax + margin,
            ymin - margin, ymax + margin,
            zmin - margin, zmax + margin
        )

        box = pv.Box(bounds=bounds)
        actor = self.plotter.add_mesh(
            box,
            style="wireframe",
            line_width=3.0,
            color=(1.0, 1.0, 0.0),  # 黄色
            opacity=1.0,
        )
        self.highlight_actor = actor
        self.plotter.render()


    def _clear_highlight_bbox(self):
        if self.highlight_actor is not None:
            try:
                self.plotter.remove_actor(self.highlight_actor)
            except Exception:
                pass
            self.highlight_actor = None


    def on_id_item_clicked(self, item: QtWidgets.QListWidgetItem):
        text = item.text()  # "id: 1"
        try:
            pid = int(text.split(":")[1].strip())
        except Exception:
            return

        multi = self.chk_multi_id.isChecked()

        if multi:
            # 複数選択モード → トグル
            if pid in self.selected_ids:
                self.selected_ids.remove(pid)
            else:
                self.selected_ids.add(pid)
        else:
            # 単一選択モード → 今のだけにする
            self.selected_ids = {pid}

        # リストの色を反映
        self._refresh_id_list_colors()

        # アクタの色を反映
        self._refresh_selected_colors()

        # 選ばれたidを黄色い箱で囲む（最後に選んだid）
        self._highlight_entry_bbox(pid)

        # 情報更新
        self.update_selected_info()

    def _refresh_id_list_colors(self):
        """idリスト側の文字色を，選択中だけ青にする"""
        for i in range(self.id_list.count()):
            item = self.id_list.item(i)
            try:
                pid = int(item.text().split(":")[1].strip())
            except Exception:
                continue
            if pid in self.selected_ids:
                item.setForeground(QtGui.QBrush(QtGui.QColor("blue")))
            else:
                item.setForeground(QtGui.QBrush(QtGui.QColor("black")))

    
    def _refresh_selected_colors(self):
        """選択されているidだけ青に，その他は赤に戻す"""
        for entry in self.plan_entries:
            pid = entry["id"]
            is_sel = pid in self.selected_ids
            color = (0, 0, 1) if is_sel else (1, 0, 0)  # blue or red

            for actor in entry["actors"]["pa"]:
                try:
                    actor.GetProperty().SetColor(color)
                except Exception:
                    pass
            for actor in entry["actors"]["contact"]:
                try:
                    actor.GetProperty().SetColor(color)
                except Exception:
                    pass
            for actor in entry["actors"]["miss"]:
                try:
                    actor.GetProperty().SetColor(color)
                except Exception:
                    pass

        self.plotter.render()

    def on_clear_selection(self):
        self.selected_ids.clear()
        self._refresh_selected_colors()
        self._refresh_id_list_colors()
        self._clear_highlight_bbox()
        self.update_selected_info()


    def update_selected_info(self):
        """右下テキストに選択IDの情報を出す"""
        lines = []
        for entry in self.plan_entries:
            pid = entry["id"]
            if pid not in self.selected_ids:
                continue
            lines.append(f"id: {pid}")

            if self.chk_info_A.isChecked():
                lines.append(f"  A: {entry['A']}")

            if self.chk_info_B.isChecked():
                lines.append(f"  B: {entry['B']}")

            if self.chk_info_Pa.isChecked():
                lines.append("  Pa:")
                for i, pa in enumerate(entry["Pa_list"], 1):
                    lines.append(f"    p{i}: ({pa[0]:.3f}, {pa[1]:.3f}, {pa[2]:.3f})")

            if self.chk_info_Pb.isChecked():
                lines.append("  Pb:")
                for i, pb in enumerate(entry["Pb_list"], 1):
                    lines.append(f"    p{i}: ({pb[0]:.3f}, {pb[1]:.3f}, {pb[2]:.3f})")

            if self.chk_info_plot.isChecked():
                lines.append("  プロット結果:")
                for res in entry["last_results"]:
                    p = res["point"]
                    lines.append(f"    p{res['pin']}: {res['status']} ({p[0]:.3f}, {p[1]:.3f}, {p[2]:.3f})")

            lines.append("")  # 区切り

        self.info_text.setPlainText("\n".join(lines))




    # ==============================
    # チェックボックス系
    # ==============================
    def on_show_stl_toggled(self, state):
        on = (state == QtCore.Qt.Checked)
        if self.mesh_actor is not None:
            self.mesh_actor.SetVisibility(on)
            self.plotter.render()

    def on_show_sim_toggled(self, state):
        on = (state == QtCore.Qt.Checked)
        for actor in self.contact_actors:
            try:
                actor.SetVisibility(on)
            except Exception:
                pass
        self.plotter.render()

    def on_show_pa_toggled(self, state):
        on = (state == QtCore.Qt.Checked)
        for actor in self.pa_actors:
            try:
                actor.SetVisibility(on)
            except Exception:
                pass
        self.plotter.render()

    def on_show_miss_toggled(self, state):
        on = (state == QtCore.Qt.Checked)
        for actor in self.miss_actors:
            try:
                actor.SetVisibility(on)
            except Exception:
                pass
        self.plotter.render()


    # ==============================
    # 視点・バウンディングボックス
    # ==============================
    def reset_view(self):
        self.plotter.view_yz()      # +Xから見た視点
        self.plotter.reset_camera()
        self.plotter.render()

    def on_bbox_toggled(self, state):
        on = (state == QtCore.Qt.Checked)
        self.toggle_bbox(on)

    def toggle_bbox(self, on: bool):
        self.remove_bbox_actors()

        if not on or self.mesh is None:
            self.dim_text.clear()
            self.plotter.render()
            return

        xmin, xmax, ymin, ymax, zmin, zmax = self.mesh.bounds
        x_len = xmax - xmin
        y_len = ymax - ymin
        z_len = zmax - zmin

        box = pv.Box(bounds=(xmin, xmax, ymin, ymax, zmin, zmax))
        self.bbox_actor = self.plotter.add_mesh(
            box, style="wireframe", line_width=2.0, color="blue", opacity=1.0
        )

        cx = (xmin + xmax) / 2.0
        cy = (ymin + ymax) / 2.0
        cz = zmin
        radius = max(x_len, y_len, z_len) * 0.01
        sphere = pv.Sphere(radius=radius, center=(cx, cy, cz))
        self.center_actor = self.plotter.add_mesh(sphere, color="red")

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
        self.dim_text.setPlainText(info)

        self.plotter.render()

    def remove_bbox_actors(self):
        for attr in ("bbox_actor", "center_actor", "scene_text_actor"):
            actor = getattr(self, attr, None)
            if actor is not None:
                try:
                    self.plotter.remove_actor(actor)
                except Exception:
                    pass
                setattr(self, attr, None)

    # ==============================
    # 終了処理
    # ==============================
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
