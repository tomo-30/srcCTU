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
from datetime import datetime
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
        self.plan_entries = []   # idごとの辞書
        self.selected_ids = set()
        self.last_clicked_id = None   # ← 一番最後にクリックしたid

        # プロット保持
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
        side_layout.setSpacing(6)
        layout.addWidget(side, stretch=0)

        # ===========================================================
        #  (A) セクション表示コントローラ（ここで表示/非表示を一括管理する）
        # ===========================================================
        ctrl_box = QtWidgets.QGroupBox("表示するセクション", self)
        ctrl_layout = QtWidgets.QHBoxLayout(ctrl_box)

        self.sec_file_chk = QtWidgets.QCheckBox("ファイル", self)
        self.sec_file_chk.setChecked(True)
        self.sec_sim_chk  = QtWidgets.QCheckBox("シミュ", self)
        self.sec_sim_chk.setChecked(True)
        self.sec_id_chk   = QtWidgets.QCheckBox("ID一覧", self)
        self.sec_id_chk.setChecked(True)
        self.sec_info_chk = QtWidgets.QCheckBox("ID情報", self)
        self.sec_info_chk.setChecked(True)

        self.sec_file_chk.toggled.connect(lambda b: self._on_section_toggled("file", b))
        self.sec_sim_chk.toggled.connect(lambda b: self._on_section_toggled("sim", b))
        self.sec_id_chk.toggled.connect(lambda b: self._on_section_toggled("id", b))
        self.sec_info_chk.toggled.connect(lambda b: self._on_section_toggled("info", b))

        ctrl_layout.addWidget(self.sec_file_chk)
        ctrl_layout.addWidget(self.sec_sim_chk)
        ctrl_layout.addWidget(self.sec_id_chk)
        ctrl_layout.addWidget(self.sec_info_chk)
        side_layout.addWidget(ctrl_box)

        # ===========================================================
        #  (B) 実際の4つのセクション
        # ===========================================================

        # ===== グループ1: ファイル・表示 =====
        self.grp_file = QtWidgets.QGroupBox("ファイル・表示", self)
        v1 = QtWidgets.QVBoxLayout(self.grp_file)

        self.lbl_stl = QtWidgets.QLabel(f"STL: {os.path.basename(self.stl_path)}", self)
        v1.addWidget(self.lbl_stl)
        self.btn_stl = QtWidgets.QPushButton("STLを選択...", self)
        self.btn_stl.clicked.connect(self.on_select_stl)
        v1.addWidget(self.btn_stl)

        self.lbl_push = QtWidgets.QLabel("push_plan: (未選択)", self)
        v1.addWidget(self.lbl_push)
        self.btn_push = QtWidgets.QPushButton("push_plan.jsonを選択...", self)
        self.btn_push.clicked.connect(self.on_select_push_plan)
        v1.addWidget(self.btn_push)

        self.btn_reset = QtWidgets.QPushButton("視点リセット (ZY正面 / XY水平)", self)
        self.btn_reset.clicked.connect(self.reset_view)
        v1.addWidget(self.btn_reset)

        self.chk_bbox = QtWidgets.QCheckBox("バウンディングボックスを表示", self)
        self.chk_bbox.stateChanged.connect(self.on_bbox_toggled)
        v1.addWidget(self.chk_bbox)

        self.chk_show_stl = QtWidgets.QCheckBox("STLを表示", self)
        self.chk_show_stl.setChecked(True)
        self.chk_show_stl.stateChanged.connect(self.on_show_stl_toggled)
        v1.addWidget(self.chk_show_stl)

        side_layout.addWidget(self.grp_file)

        # ===== グループ2: シミュレーション =====
        self.grp_sim = QtWidgets.QGroupBox("シミュレーション", self)
        v2 = QtWidgets.QVBoxLayout(self.grp_sim)

        self.chk_show_sim = QtWidgets.QCheckBox("シミュレーション結果を表示", self)
        self.chk_show_sim.setChecked(True)
        self.chk_show_sim.stateChanged.connect(self.on_show_sim_toggled)
        v2.addWidget(self.chk_show_sim)

        self.chk_show_pa = QtWidgets.QCheckBox("ピン押下スタート(Pa)を表示", self)
        self.chk_show_pa.setChecked(True)
        self.chk_show_pa.stateChanged.connect(self.on_show_pa_toggled)
        v2.addWidget(self.chk_show_pa)

        self.chk_show_miss = QtWidgets.QCheckBox("未接触ピンの終端(Pb)を表示", self)
        self.chk_show_miss.setChecked(True)
        self.chk_show_miss.stateChanged.connect(self.on_show_miss_toggled)
        v2.addWidget(self.chk_show_miss)

        self.chk_real = QtWidgets.QCheckBox("現実モード(ピン長さ制限)", self)
        self.chk_real.setChecked(False)
        self.chk_real.stateChanged.connect(self.on_real_mode_toggled)
        v2.addWidget(self.chk_real)

        hbox_pin = QtWidgets.QHBoxLayout()
        hbox_pin.addWidget(QtWidgets.QLabel("ピン長さ[mm]:", self))
        self.spin_pinlen = QtWidgets.QDoubleSpinBox(self)
        self.spin_pinlen.setRange(1.0, 500.0)
        self.spin_pinlen.setValue(30.0)
        self.spin_pinlen.setSingleStep(1.0)
        self.spin_pinlen.valueChanged.connect(self.on_pinlen_changed)
        hbox_pin.addWidget(self.spin_pinlen)
        v2.addLayout(hbox_pin)

        self.btn_sim = QtWidgets.QPushButton("接触シミュレーション開始", self)
        self.btn_sim.clicked.connect(self.on_run_simulation)
        v2.addWidget(self.btn_sim)

        side_layout.addWidget(self.grp_sim)

        # ===== グループ3: ID選択 =====
        self.grp_id = QtWidgets.QGroupBox("ID選択", self)
        v3 = QtWidgets.QVBoxLayout(self.grp_id)

        self.chk_multi_id = QtWidgets.QCheckBox("idを複数選択する", self)
        self.chk_multi_id.setChecked(True)
        v3.addWidget(self.chk_multi_id)

        v3.addWidget(QtWidgets.QLabel("ID一覧:", self))
        self.id_list = QtWidgets.QListWidget(self)
        self.id_list.itemClicked.connect(self.on_id_item_clicked)
        self.id_list.setMinimumHeight(120)
        v3.addWidget(self.id_list)

        self.btn_clear_sel = QtWidgets.QPushButton("選択をクリア", self)
        self.btn_clear_sel.clicked.connect(self.on_clear_selection)
        v3.addWidget(self.btn_clear_sel)

        side_layout.addWidget(self.grp_id)

        # ===== グループ4: ID情報 =====
        self.grp_info = QtWidgets.QGroupBox("ID情報", self)
        v4 = QtWidgets.QVBoxLayout(self.grp_info)

        self.chk_info_A = QtWidgets.QCheckBox("Aの座標", self); self.chk_info_A.setChecked(True)
        self.chk_info_B = QtWidgets.QCheckBox("Bの座標", self); self.chk_info_B.setChecked(True)
        self.chk_info_Pa = QtWidgets.QCheckBox("Paの座標群", self); self.chk_info_Pa.setChecked(False)
        self.chk_info_Pb = QtWidgets.QCheckBox("Pbの座標群", self); self.chk_info_Pb.setChecked(False)
        self.chk_info_plot = QtWidgets.QCheckBox("各ピンのプロット座標", self); self.chk_info_plot.setChecked(True)
        for w in (self.chk_info_A, self.chk_info_B, self.chk_info_Pa,
                  self.chk_info_Pb, self.chk_info_plot):
            w.stateChanged.connect(self.update_selected_info)
            v4.addWidget(w)

        self.info_text = QtWidgets.QTextEdit(self)
        self.info_text.setReadOnly(True)
        self.info_text.setSizePolicy(QtWidgets.QSizePolicy.Expanding,
                                     QtWidgets.QSizePolicy.Expanding)
        v4.addWidget(self.info_text)

        side_layout.addWidget(self.grp_info)

        # 一番下：ログ・寸法
        self.dim_text = QtWidgets.QTextEdit(self)
        self.dim_text.setReadOnly(True)
        self.dim_text.setFixedHeight(90)
        side_layout.addWidget(self.dim_text)

        side_layout.addStretch(1)

        # 保存先（初期: 実行ファイルと同じ場所）
        self.save_dir = self.current_dir

        save_box = QtWidgets.QGroupBox("結果保存", self)
        save_layout = QtWidgets.QVBoxLayout(save_box)

        # 保存先表示
        self.lbl_save_dir = QtWidgets.QLabel(f"保存先: {self.save_dir}", self)
        self.lbl_save_dir.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        save_layout.addWidget(self.lbl_save_dir)

        # 保存先変更ボタン
        btn_change_dir = QtWidgets.QPushButton("保存先を変更...", self)
        btn_change_dir.clicked.connect(self.on_change_save_dir)
        save_layout.addWidget(btn_change_dir)

        # JSON保存ボタン
        btn_save_json = QtWidgets.QPushButton("シミュレーション結果をJSON保存", self)
        btn_save_json.clicked.connect(self.on_save_simulation_json)
        save_layout.addWidget(btn_save_json)

        v2.addWidget(save_box)


        # 初期読み込み
        self.load_mesh_and_setup(self.stl_path)

    def _on_section_toggled(self, name: str, checked: bool):
        """上部のチェックで各セクションの表示/非表示を切り替える（値は保持）"""
        if name == "file":
            self.grp_file.setVisible(checked)
        elif name == "sim":
            self.grp_sim.setVisible(checked)
        elif name == "id":
            self.grp_id.setVisible(checked)
        elif name == "info":
            self.grp_info.setVisible(checked)

        # レイアウトを再計算
        self._update_side_heights()

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
                # すでに選択中なら解除
                self.selected_ids.remove(pid)
            else:
                self.selected_ids.add(pid)
        else:
            # 単一選択モードでもトグルにする
            if len(self.selected_ids) == 1 and pid in self.selected_ids:
                # 1つだけ選ばれていて，それをもう一度押した → 全解除
                self.selected_ids.clear()
            else:
                # それ以外はこのidだけにする
                self.selected_ids = {pid}

        # 「一番最後にクリックしたid」は，選択が残っている場合だけ記録
        if pid in self.selected_ids:
            self.last_clicked_id = pid
        else:
            if self.selected_ids:
                self.last_clicked_id = list(self.selected_ids)[-1]
            else:
                self.last_clicked_id = None

        # リスト側の色更新
        self._refresh_id_list_colors()
        # アクタの色更新（青/赤）
        self._refresh_selected_colors()

        # ハイライトの更新
        if self.last_clicked_id is not None:
            entry = self._find_entry_by_id(self.last_clicked_id)
            if entry is not None:
                # 再描画して一番手前に
                self._redraw_entry_actors(entry, (0, 0, 1))
                # 今表示されているプロットだけで囲む
                self._highlight_entry_bbox(self.last_clicked_id)
        else:
            # 何も選ばれてなければ囲いを消す
            self._clear_highlight_bbox()

        # 情報更新
        self.update_selected_info()

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

    # 保存系
    def on_change_save_dir(self):
        d = QtWidgets.QFileDialog.getExistingDirectory(
            self, "保存先フォルダを選択", self.save_dir
        )
        if d:
            self.save_dir = d
            self.lbl_save_dir.setText(f"保存先: {self.save_dir}")

    def on_save_simulation_json(self):
        """現在のシミュレーション結果をJSONに保存する"""
        if not self.plan_entries:
            QtWidgets.QMessageBox.warning(self, "警告", "push_plan.jsonを先に読み込んでください．")
            return

        # どれか1回はシミュレーションを実行して entry['last_results'] を持っている必要がある
        any_result = any(len(e.get("last_results", [])) > 0 for e in self.plan_entries)
        if not any_result:
            QtWidgets.QMessageBox.warning(self, "警告", "シミュレーション結果がありません．先に実行してください．")
            return

        data = self._assemble_simulation_result_dict()

        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        fname = f"sim_result_{ts}.json"
        outpath = os.path.join(self.save_dir, fname)

        try:
            with open(outpath, "w", encoding="utf-8") as f:
                json.dump(data, f, ensure_ascii=False, indent=2)
            QtWidgets.QMessageBox.information(self, "保存完了", f"保存しました:\n{outpath}")
        except Exception as e:
            QtWidgets.QMessageBox.critical(self, "保存エラー", f"保存に失敗しました:\n{e}")

    def _assemble_simulation_result_dict(self) -> dict:
        """push_plan + 実際のプロット結果(接触/未接触と座標) を1つのdictにまとめる"""
        meta = {
            "timestamp": datetime.now().isoformat(timespec="seconds"),
            "mode": "realistic" if self.sim_realistic else "ideal",
            "pin_length_mm": float(self.pin_length_mm),
            "stl_file": os.path.basename(self.stl_path) if self.stl_path else None,
            "push_plan_file": os.path.basename(self.push_plan_path) if self.push_plan_path else None,
        }

        plans_out = []
        for entry in self.plan_entries:
            pid = entry.get("id")
            A = entry.get("A", [])
            B = entry.get("B", [])
            Pa_list = entry.get("Pa_list", [])
            Pb_list = entry.get("Pb_list", [])
            results = []
            # entry["last_results"]: {pin, status('〇'/'×'), point:(x,y,z)}
            for item in entry.get("last_results", []):
                p = item.get("point", (None, None, None))
                results.append({
                    "pin": int(item.get("pin", 0)),
                    "contact": True if item.get("status") == "〇" else False,
                    "point": [float(p[0]), float(p[1]), float(p[2])] if p[0] is not None else [None, None, None],
                })

            plans_out.append({
                "id": pid,
                "A": A,
                "B": B,
                "Pa": Pa_list,
                "Pb": Pb_list,
                "results": results,   # 実際に到達した先端座標＆接触有無
            })

        out = {
            "meta": meta,
            "plan": plans_out,
        }
        return out

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
        for entry in self.plan_entries:
            entry["actors"]["contact"] = []
            entry["actors"]["miss"] = []
            entry["last_results"] = []

        # 実行
        if self.sim_realistic:
            self._run_simulation_realistic()
        else:
            self._run_simulation_ideal()

        # 色を再反映
        self._refresh_selected_colors()
        self._refresh_id_list_colors()

        # もし最後に選んだidがあれば，それを前面に出して囲い直す
        if self.last_clicked_id is not None and self.last_clicked_id in self.selected_ids:
            entry = self._find_entry_by_id(self.last_clicked_id)
            if entry is not None:
                self._redraw_entry_actors(entry, (0, 0, 1))
                self._highlight_entry_bbox(self.last_clicked_id)

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
        """指定idの『現在表示されている』プロットだけで最小直方体を描画する"""
        # 先に古いのを消す
        self._clear_highlight_bbox()

        entry = self._find_entry_by_id(pid)
        if entry is None:
            return

        pts = []

        # Paが表示中ならPaを追加
        if self.chk_show_pa.isChecked():
            for pa in entry["Pa_list"]:
                pts.append((pa[0], pa[1], pa[2]))

        # シミュ結果：表示されているものだけ
        for res in entry.get("last_results", []):
            if res["status"] == "〇":
                if self.chk_show_sim.isChecked():
                    pts.append(res["point"])
            else:
                if self.chk_show_miss.isChecked():
                    pts.append(res["point"])

        if not pts:
            return

        pts = np.asarray(pts, dtype=float)
        mins = pts.min(axis=0)
        maxs = pts.max(axis=0)

        margin = 5.0
        xmin, ymin, zmin = mins - margin
        xmax, ymax, zmax = maxs + margin

        box = pv.Box(bounds=(xmin, xmax, ymin, ymax, zmin, zmax))
        actor = self.plotter.add_mesh(
            box,
            style="wireframe",
            line_width=4.0,
            color=(1.0, 0.4, 0.0),  # ちょっと濃い黄橙
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


    def _on_section_toggled(self, name: str, checked: bool):
        """上部のチェックで各セクションの表示/非表示を切り替える（値は保持）"""
        if name == "file":
            self.grp_file.setVisible(checked)
        elif name == "sim":
            self.grp_sim.setVisible(checked)
        elif name == "id":
            self.grp_id.setVisible(checked)
        elif name == "info":
            self.grp_info.setVisible(checked)

        # レイアウトを再計算
        self._update_side_heights()

    def _refresh_id_list_colors(self):
        """ID一覧の文字色を選択状態に応じて更新する"""
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

    def _update_side_heights(self):
        """グループの開閉に応じて右側の情報欄のレイアウトを更新する"""
        self.info_text.updateGeometry()
        self.info_text.repaint()

    def _find_entry_by_id(self, pid: int):
        for entry in self.plan_entries:
            if entry["id"] == pid:
                return entry
        return None

    def _redraw_entry_actors(self, entry, color):
        """
        指定entryに属する現状の表示をすべて描き直して，一番手前に出す．
        color: (r,g,b)
        """
        radius = self._contact_radius_from_mesh()
        miss_size = radius * 2.0  # ×の大きさ

        # いったん消す
        for key in ("pa", "contact", "miss"):
            for actor in entry["actors"][key]:
                try:
                    self.plotter.remove_actor(actor)
                except Exception:
                    pass
            entry["actors"][key] = []

        # Pa（表示ONのときだけ）
        if self.chk_show_pa.isChecked():
            for pa in entry["Pa_list"]:
                sp = pv.Sphere(radius=radius, center=(pa[0], pa[1], pa[2]))
                actor = self.plotter.add_mesh(
                    sp,
                    color=color,
                    style="wireframe",
                    line_width=2.0,
                )
                entry["actors"]["pa"].append(actor)
                self.pa_actors.append(actor)

        # シミュ結果（表示ONのものだけ）
        for res in entry.get("last_results", []):
            pt = res["point"]
            if res["status"] == "〇":
                if self.chk_show_sim.isChecked():
                    sp = pv.Sphere(radius=radius, center=pt)
                    actor = self.plotter.add_mesh(sp, color=color)
                    entry["actors"]["contact"].append(actor)
                    self.contact_actors.append(actor)
            else:
                if self.chk_show_miss.isChecked():
                    actors = self._add_cross_at(pt, miss_size)
                    for a in actors:
                        a.GetProperty().SetColor(color)
                        entry["actors"]["miss"].append(a)
                        self.miss_actors.append(a)

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
