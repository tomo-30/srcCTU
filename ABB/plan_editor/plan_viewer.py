# -*- coding: utf-8 -*-
"""
plan_viewer.py
JSONで作成された push_plan.json を「逆表示」するビューア

想定するJSON形式（例）:
{
  "plan": [
    {
      "id": 1,
      "A": [Ax, Ay, Az, Ar, Ap, Ayaw],
      "B": [Bx, By, Bz, Br, Bp, Byaw],
      "Pa": [[x,y,z], ...],   # N個
      "Pb": [[x,y,z], ...]    # N個
    },
    ...
  ]
}

機能:
- push_plan.json の読み込み（起動時に同階層を試す／ボタンで再選択可）
- IDごとの表示／全ID表示の切替
- A/B/Pa/Pb の個別表示ON/OFF
- 2D/3D切替
- A-Bを線で結んで表示
"""

import os
import json
import sys

from PyQt5 import QtWidgets, QtCore

import matplotlib
matplotlib.use("Qt5Agg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas


HERE = os.path.abspath(os.path.dirname(__file__))
DEFAULT_JSON = os.path.join(HERE, "push_plan.json")


class PlanViewer(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)

        self.setWindowTitle("push_plan.json Viewer")
        self.resize(1100, 750)

        # 内部データ
        self.plan_data = []   # [{"id":..., "A":[...], "B":[...], "Pa":[...], "Pb":[...]}]
        self.current_json_path = ""

        # ----------------- 左ペイン（操作UI） -----------------
        left_layout = QtWidgets.QVBoxLayout()

        # JSONパス行
        path_layout = QtWidgets.QHBoxLayout()
        self.path_edit = QtWidgets.QLineEdit()
        self.path_edit.setReadOnly(True)
        btn_open = QtWidgets.QPushButton("開く…")
        btn_open.clicked.connect(self.on_open_json)
        path_layout.addWidget(self.path_edit, 1)
        path_layout.addWidget(btn_open, 0)
        left_layout.addLayout(path_layout)

        # IDリスト
        self.list_ids = QtWidgets.QListWidget()
        self.list_ids.itemSelectionChanged.connect(self.on_id_changed)
        left_layout.addWidget(QtWidgets.QLabel("Plan ID 一覧"))
        left_layout.addWidget(self.list_ids, 1)

        # 全ID表示
        self.cb_show_all = QtWidgets.QCheckBox("全IDを表示する")
        self.cb_show_all.setChecked(True)
        self.cb_show_all.stateChanged.connect(self.update_plot)
        left_layout.addWidget(self.cb_show_all)

        # 表示項目
        left_layout.addWidget(QtWidgets.QLabel("表示項目"))
        self.cb_show_A = QtWidgets.QCheckBox("A点を表示")
        self.cb_show_A.setChecked(True)
        self.cb_show_B = QtWidgets.QCheckBox("B点を表示")
        self.cb_show_B.setChecked(True)
        self.cb_show_Pa = QtWidgets.QCheckBox("Pa群を表示")
        self.cb_show_Pa.setChecked(True)
        self.cb_show_Pb = QtWidgets.QCheckBox("Pb群を表示")
        self.cb_show_Pb.setChecked(True)
        for cb in (self.cb_show_A, self.cb_show_B, self.cb_show_Pa, self.cb_show_Pb):
            cb.stateChanged.connect(self.update_plot)
            left_layout.addWidget(cb)

        # 2D/3D切替
        left_layout.addWidget(QtWidgets.QLabel("表示モード"))
        self.rb_2d = QtWidgets.QRadioButton("2D (XY)")
        self.rb_3d = QtWidgets.QRadioButton("3D (XYZ)")
        self.rb_2d.setChecked(True)
        self.rb_2d.toggled.connect(self.update_plot)
        self.rb_3d.toggled.connect(self.update_plot)
        left_layout.addWidget(self.rb_2d)
        left_layout.addWidget(self.rb_3d)

        # 凡例表示
        self.cb_legend = QtWidgets.QCheckBox("凡例を表示")
        self.cb_legend.setChecked(True)
        self.cb_legend.stateChanged.connect(self.update_plot)
        left_layout.addWidget(self.cb_legend)

        # ストレッチ
        left_layout.addStretch(1)

        # ----------------- 右側（matplotlib） -----------------
        self.fig = plt.figure(figsize=(6, 6))
        self.canvas = FigureCanvas(self.fig)
        # 最初は2D
        self.ax = self.fig.add_subplot(111)

        # ----------------- 全体レイアウト -----------------
        main_layout = QtWidgets.QHBoxLayout(self)
        main_layout.addLayout(left_layout, 0)
        main_layout.addWidget(self.canvas, 1)

        # 起動時にデフォルトJSONを試す
        self.try_load_default()

    # ------------------------------------------------------------
    # JSON読み込み関係
    # ------------------------------------------------------------
    def try_load_default(self):
        if os.path.exists(DEFAULT_JSON):
            self.load_json(DEFAULT_JSON)
        else:
            # 何もなければ空表示
            self.path_edit.setText("(push_plan.json が見つかりません)")
            self.update_plot()

    def on_open_json(self):
        path, _ = QtWidgets.QFileDialog.getOpenFileName(
            self,
            "push_plan.json を選択",
            HERE,
            "JSON files (*.json);;All files (*.*)"
        )
        if path:
            self.load_json(path)

    def load_json(self, path: str):
        try:
            with open(path, "r", encoding="utf-8") as f:
                data = json.load(f)
        except Exception as e:
            QtWidgets.QMessageBox.critical(self, "読み込みエラー", str(e))
            return

        # 想定: {"plan": [ ... ]}
        plan_list = data.get("plan", [])
        if not isinstance(plan_list, list):
            QtWidgets.QMessageBox.critical(self, "形式エラー", "JSON内に 'plan' リストがありません．")
            return

        self.plan_data = plan_list
        self.current_json_path = path
        self.path_edit.setText(os.path.relpath(path, HERE))

        # IDリストを作る
        self.list_ids.blockSignals(True)
        self.list_ids.clear()
        for item in self.plan_data:
            pid = item.get("id", None)
            lw_item = QtWidgets.QListWidgetItem(str(pid))
            self.list_ids.addItem(lw_item)
        self.list_ids.blockSignals(False)

        # 先頭を選択（ただし「全ID表示」がONなら描画には使わない）
        if self.list_ids.count() > 0:
            self.list_ids.setCurrentRow(0)

        # 描画更新
        self.update_plot()

    # ------------------------------------------------------------
    # イベント
    # ------------------------------------------------------------
    def on_id_changed(self):
        # ID選択が変わった場合
        if not self.cb_show_all.isChecked():
            self.update_plot()

    # ------------------------------------------------------------
    # 描画
    # ------------------------------------------------------------
    def ensure_axes(self, is3d: bool):
        self.fig.clf()
        if is3d:
            self.ax = self.fig.add_subplot(111, projection="3d")
        else:
            self.ax = self.fig.add_subplot(111)

    def update_plot(self):
        is3d = self.rb_3d.isChecked()
        self.ensure_axes(is3d)

        # 表示対象のplanを決定
        targets = []
        if self.cb_show_all.isChecked():
            targets = self.plan_data
        else:
            # 選択されているIDだけ
            sel_items = self.list_ids.selectedItems()
            if sel_items:
                sel_id_str = sel_items[0].text()
                try:
                    sel_id = int(sel_id_str)
                except ValueError:
                    sel_id = sel_id_str
                for item in self.plan_data:
                    if item.get("id") == sel_id:
                        targets = [item]
                        break

        # 実際に描画
        for item in targets:
            self.draw_one_plan(item, is3d=is3d)

        # 軸設定
        if is3d:
            self.ax.set_xlabel("X [mm]")
            self.ax.set_ylabel("Y [mm]")
            self.ax.set_zlabel("Z [mm]")
            self.ax.set_title("push_plan 3D viewer")
            # 軸範囲を多少自動調整
            # →描いた後でもう一度limを決めたければここでやる
        else:
            self.ax.set_aspect("equal", adjustable="box")
            self.ax.set_xlabel("X [mm]")
            self.ax.set_ylabel("Y [mm]")
            self.ax.grid(True, linestyle=":")
            self.ax.set_title("push_plan 2D viewer (XY)")

        if self.cb_legend.isChecked():
            self.ax.legend(loc="best")

        self.canvas.draw()

    def draw_one_plan(self, item: dict, is3d: bool):
        pid = item.get("id", "?")

        # --- A点 ---
        if self.cb_show_A.isChecked():
            A = item.get("A")
            if isinstance(A, list) and len(A) >= 3:
                Ax, Ay, Az = A[0], A[1], A[2]
                if is3d:
                    self.ax.scatter([Ax], [Ay], [Az],
                                    color="green", s=60,
                                    label=f"A (id={pid})")
                else:
                    self.ax.scatter([Ax], [Ay],
                                    color="green", s=60,
                                    label=f"A (id={pid})")

        # --- B点 ---
        if self.cb_show_B.isChecked():
            B = item.get("B")
            if isinstance(B, list) and len(B) >= 3:
                Bx, By, Bz = B[0], B[1], B[2]
                if is3d:
                    self.ax.scatter([Bx], [By], [Bz],
                                    color="blue", s=60,
                                    label=f"B (id={pid})")
                else:
                    self.ax.scatter([Bx], [By],
                                    color="blue", s=60,
                                    label=f"B (id={pid})")

        # --- A-Bを線で結ぶ（あるとわかりやすい）---
        A = item.get("A")
        B = item.get("B")
        if A and B and len(A) >= 3 and len(B) >= 3:
            Ax, Ay, Az = A[0], A[1], A[2]
            Bx, By, Bz = B[0], B[1], B[2]
            if is3d:
                self.ax.plot([Ax, Bx], [Ay, By], [Az, Bz],
                             color="gray", linestyle="--", linewidth=1)
            else:
                self.ax.plot([Ax, Bx], [Ay, By],
                             color="gray", linestyle="--", linewidth=1)

        # --- Pa ---
        if self.cb_show_Pa.isChecked():
            Pa = item.get("Pa", [])
            if isinstance(Pa, list) and len(Pa) > 0:
                xs = [p[0] for p in Pa]
                ys = [p[1] for p in Pa]
                zs = [p[2] for p in Pa]
                if is3d:
                    self.ax.scatter(xs, ys, zs,
                                    facecolors="none",
                                    edgecolors="purple",
                                    s=50,
                                    label=f"Pa (id={pid})")
                else:
                    self.ax.scatter(xs, ys,
                                    facecolors="none",
                                    edgecolors="purple",
                                    s=50,
                                    label=f"Pa (id={pid})")

        # --- Pb ---
        if self.cb_show_Pb.isChecked():
            Pb = item.get("Pb", [])
            if isinstance(Pb, list) and len(Pb) > 0:
                xs = [p[0] for p in Pb]
                ys = [p[1] for p in Pb]
                zs = [p[2] for p in Pb]
                if is3d:
                    self.ax.scatter(xs, ys, zs,
                                    facecolors="none",
                                    edgecolors="red",
                                    s=50,
                                    label=f"Pb (id={pid})")
                else:
                    self.ax.scatter(xs, ys,
                                    facecolors="none",
                                    edgecolors="red",
                                    s=50,
                                    label=f"Pb (id={pid})")


def main():
    app = QtWidgets.QApplication(sys.argv)
    w = PlanViewer()
    w.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
