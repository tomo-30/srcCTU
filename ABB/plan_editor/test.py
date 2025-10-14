# -*- coding: utf-8 -*-
# gui_push_plan_v3_spiral.py
# 変更点:
#  - ID付与順を中心(1)→上→右→下→左…の外向き正方らせんに変更
#  - 既存仕様: 「プロットのみ更新＝2D(Bのみ/自動フィット)」「生成＆保存＝JSON上書き→3D(A/B表示)」
#  - push_plan.json はスクリプトと同じフォルダに上書き保存

import json
import math
import os
import tkinter as tk
from tkinter import ttk, messagebox

import matplotlib
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401


def spiral_indices(nx: int, ny: int):
    """
    中心(0,0)から，上→右→下→左の順で外向きらせんに巡回する格子インデックス列を返す．
    有効範囲は ix∈[-nx,nx], iy∈[-ny,ny]．
    """
    total = (2 * nx + 1) * (2 * ny + 1)
    ix = iy = 0
    yield (ix, iy)
    count = 1

    # 方向: 上, 右, 下, 左
    dirs = [(0, 1), (1, 0), (0, -1), (-1, 0)]
    step_len = 1

    while count < total:
        for d_idx, (dxi, dyi) in enumerate(dirs):
            # 同じステップ長で2方向ごとに step_len を+1
            steps = step_len
            for _ in range(steps):
                ix += dxi
                iy += dyi
                if (-nx <= ix <= nx) and (-ny <= iy <= ny):
                    yield (ix, iy)
                    count += 1
                    if count >= total:
                        return
            if d_idx % 2 == 1:
                step_len += 1


def generate_points(params):
    """
    入力パラメータからB点（XY格子，IDはらせん順）とA点（BをdzだけZ方向へ）を生成．
    戻り値:
      records: JSON保存用 list[dict]（id/type/x/y/z/roll/pitch/yaw）
      plot_pairs_2d: list[(id,(xB,yB))]  …2D描画用
      points_3d: dict{"A": [(id,x,y,z)], "B": [(id,x,y,z)]} …3D描画用
    """
    Ox = params["Ox"]; Oy = params["Oy"]; Oz = params["Oz"]
    lx = params["lx"]; ly = params["ly"]
    dx = params["dx"]; dy = params["dy"]
    dz = params["dz"]

    # 姿勢（度）
    Broll = params["Broll"]; Bpitch = params["Bpitch"]; Byaw = params["Byaw"]
    Aroll = params["Aroll"]; Apitch = params["Apitch"]; Ayaw = params["Ayaw"]

    if dx <= 0 or dy <= 0:
        raise ValueError("dx, dy は正の数である必要があります")
    if lx <= 0 or ly <= 0:
        raise ValueError("lx, ly は正の数である必要があります")

    # 中心からの格子半径（インデックス範囲）
    nx = int(math.floor((lx / 2.0) / dx))
    ny = int(math.floor((ly / 2.0) / dy))

    # ===== らせん順で格子点（インデックス）を列挙 =====
    records = []
    plot_pairs_2d = []
    ptsA, ptsB = [], []
    cur_id = 1

    for ix, iy in spiral_indices(nx, ny):
        x = Ox + ix * dx
        y = Oy + iy * dy
        # 理論上，上の範囲条件で必ず長方形内だが，安全のためチェックを残す
        if abs(x - Ox) <= lx / 2.0 + 1e-9 and abs(y - Oy) <= ly / 2.0 + 1e-9:
            recB = {
                "id": cur_id, "type": "B",
                "x": float(x), "y": float(y), "z": float(Oz),
                "roll": float(Broll), "pitch": float(Bpitch), "yaw": float(Byaw),
            }
            recA = {
                "id": cur_id, "type": "A",
                "x": float(x), "y": float(y), "z": float(Oz + dz),
                "roll": float(Aroll), "pitch": float(Apitch), "yaw": float(Ayaw),
            }
            records.append(recB)
            records.append(recA)

            plot_pairs_2d.append((cur_id, (x, y)))
            ptsB.append((cur_id, x, y, Oz))
            ptsA.append((cur_id, x, y, Oz + dz))
            cur_id += 1

    points_3d = {"A": ptsA, "B": ptsB}
    return records, plot_pairs_2d, points_3d


class PushPlanGUI(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Push Plan Generator (A/B points) — Spiral ID")

        # ===== 左：入力フォーム =====
        frm = ttk.Frame(self, padding=8)
        frm.grid(row=0, column=0, sticky="nsew")
        self.columnconfigure(1, weight=1)
        self.rowconfigure(0, weight=1)

        self.var_Ox = tk.DoubleVar(value=400.0)
        self.var_Oy = tk.DoubleVar(value=0.0)
        self.var_Oz = tk.DoubleVar(value=150.0)

        self.var_lx = tk.DoubleVar(value=200.0)
        self.var_ly = tk.DoubleVar(value=200.0)

        self.var_dx = tk.DoubleVar(value=20.0)
        self.var_dy = tk.DoubleVar(value=20.0)

        self.var_dz = tk.DoubleVar(value=250.0)

        self.var_Broll = tk.DoubleVar(value=0.0)
        self.var_Bpitch = tk.DoubleVar(value=180.0)
        self.var_Byaw = tk.DoubleVar(value=0.0)

        self.var_Aroll = tk.DoubleVar(value=0.0)
        self.var_Apitch = tk.DoubleVar(value=180.0)
        self.var_Ayaw = tk.DoubleVar(value=0.0)

        row = 0
        ttk.Label(frm, text="中心点 O [mm]").grid(row=row, column=0, sticky="w"); row += 1
        self._add_labeled_entry(frm, "Ox", self.var_Ox, row); row += 1
        self._add_labeled_entry(frm, "Oy", self.var_Oy, row); row += 1
        self._add_labeled_entry(frm, "Oz", self.var_Oz, row); row += 1

        ttk.Separator(frm).grid(row=row, column=0, columnspan=2, sticky="ew", pady=4); row += 1

        ttk.Label(frm, text="XY長方形 [mm]").grid(row=row, column=0, sticky="w"); row += 1
        self._add_labeled_entry(frm, "lx", self.var_lx, row); row += 1
        self._add_labeled_entry(frm, "ly", self.var_ly, row); row += 1

        ttk.Separator(frm).grid(row=row, column=0, columnspan=2, sticky="ew", pady=4); row += 1

        ttk.Label(frm, text="グリッド間隔 [mm]").grid(row=row, column=0, sticky="w"); row += 1
        self._add_labeled_entry(frm, "dx", self.var_dx, row); row += 1
        self._add_labeled_entry(frm, "dy", self.var_dy, row); row += 1

        ttk.Separator(frm).grid(row=row, column=0, columnspan=2, sticky="ew", pady=4); row += 1

        self._add_labeled_entry(frm, "dz (AのZ=BのZ+dz)", self.var_dz, row); row += 1

        ttk.Separator(frm).grid(row=row, column=0, columnspan=2, sticky="ew", pady=4); row += 1

        ttk.Label(frm, text="姿勢（度）: B点").grid(row=row, column=0, sticky="w"); row += 1
        self._add_labeled_entry(frm, "B roll", self.var_Broll, row); row += 1
        self._add_labeled_entry(frm, "B pitch", self.var_Bpitch, row); row += 1
        self._add_labeled_entry(frm, "B yaw", self.var_Byaw, row); row += 1

        ttk.Label(frm, text="姿勢（度）: A点").grid(row=row, column=0, sticky="w"); row += 1
        self._add_labeled_entry(frm, "A roll", self.var_Aroll, row); row += 1
        self._add_labeled_entry(frm, "A pitch", self.var_Apitch, row); row += 1
        self._add_labeled_entry(frm, "A yaw", self.var_Ayaw, row); row += 1

        btn_box = ttk.Frame(frm)
        btn_box.grid(row=row, column=0, columnspan=2, pady=8, sticky="ew")
        ttk.Button(btn_box, text="生成＆保存（JSON上書き→3D表示）", command=self.on_generate_save)\
            .grid(row=0, column=0, padx=4)
        ttk.Button(btn_box, text="プロットのみ更新（2D）", command=self.on_plot_only_update)\
            .grid(row=0, column=1, padx=4)

        # ===== 右：プロット領域 =====
        plot_frame = ttk.Frame(self, padding=4)
        plot_frame.grid(row=0, column=1, sticky="nsew")
        self._build_canvas(plot_frame)

        # データ保持
        self.records = []
        self.plot_pairs_2d = []
        self.points_3d = {"A": [], "B": []}

        # スクリプト格納ディレクトリ（JSON保存先）
        self.script_dir = os.path.dirname(os.path.abspath(__file__))

        # 初期は2Dでプロット
        self.on_plot_only_update()

    def _add_labeled_entry(self, parent, label, var, row):
        frm = ttk.Frame(parent)
        frm.grid(row=row, column=0, columnspan=2, sticky="ew", pady=2)
        ttk.Label(frm, text=label, width=20).pack(side="left")
        e = ttk.Entry(frm, textvariable=var, width=12)
        e.pack(side="left")

    def _collect_params(self):
        return dict(
            Ox=self.var_Ox.get(), Oy=self.var_Oy.get(), Oz=self.var_Oz.get(),
            lx=self.var_lx.get(), ly=self.var_ly.get(),
            dx=self.var_dx.get(), dy=self.var_dy.get(),
            dz=self.var_dz.get(),
            Broll=self.var_Broll.get(), Bpitch=self.var_Bpitch.get(), Byaw=self.var_Byaw.get(),
            Aroll=self.var_Aroll.get(), Apitch=self.var_Apitch.get(), Ayaw=self.var_Ayaw.get(),
        )

    def _build_canvas(self, parent):
        self.fig = Figure(figsize=(6, 6), dpi=100)
        self.ax = self.fig.add_subplot(111)
        self.ax.set_aspect("equal", adjustable="box")
        self.ax.grid(True, linestyle="--", linewidth=0.5)
        self.ax.set_xlabel("X [mm]")
        self.ax.set_ylabel("Y [mm]")
        self.canvas = FigureCanvasTkAgg(self.fig, master=parent)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill="both", expand=True)

    def _switch_to_2d(self):
        self.fig.clf()
        self.ax = self.fig.add_subplot(111)
        self.ax.set_aspect("equal", adjustable="box")
        self.ax.grid(True, linestyle="--", linewidth=0.5)
        self.ax.set_xlabel("X [mm]")
        self.ax.set_ylabel("Y [mm]")

    def _switch_to_3d(self):
        self.fig.clf()
        self.ax = self.fig.add_subplot(111, projection="3d")
        self.ax.set_xlabel("X [mm]")
        self.ax.set_ylabel("Y [mm]")
        self.ax.set_zlabel("Z [mm]")
        self.ax.view_init(elev=22, azim=-60)

    # ---- Buttons ----
    def on_plot_only_update(self):
        """保存せず，現在パラメータで再生成 → 2D表示（Bのみ，範囲はB全点にフィット）"""
        try:
            params = self._collect_params()
            self.records, self.plot_pairs_2d, self.points_3d = generate_points(params)
            self.update_plot_2d(params)
        except Exception as e:
            messagebox.showerror("エラー", str(e))

    def on_generate_save(self):
        """JSON保存 → 3D表示に切替してA/Bを両方表示"""
        try:
            params = self._collect_params()
            self.records, self.plot_pairs_2d, self.points_3d = generate_points(params)
            out_path = os.path.join(self.script_dir, "push_plan.json")  # スクリプトと同じ場所
            with open(out_path, "w", encoding="utf-8") as f:
                json.dump(self.records, f, ensure_ascii=False, indent=2)
            self.update_plot_3d(params)
        except Exception as e:
            messagebox.showerror("エラー", str(e))

    # ---- Plotters ----
    def update_plot_2d(self, params):
        """2D描画（Bのみ，範囲はB全点にフィット）"""
        self._switch_to_2d()
        Ox, Oy, lx, ly = params["Ox"], params["Oy"], params["lx"], params["ly"]

        # XY長方形の枠を参考表示
        rect_x = [Ox - lx/2, Ox + lx/2, Ox + lx/2, Ox - lx/2, Ox - lx/2]
        rect_y = [Oy - ly/2, Oy - ly/2, Oy + ly/2, Oy + ly/2, Oy - ly/2]
        self.ax.plot(rect_x, rect_y, linestyle=":", linewidth=1)

        n = len(self.plot_pairs_2d)
        xs, ys = [], []
        for pid, (xB, yB) in self.plot_pairs_2d:
            alpha = 1.0 - 0.8 * ((pid - 1) / max(1, n - 1))
            self.ax.scatter([xB], [yB], s=40, edgecolors="none", c="tab:blue", alpha=alpha, zorder=3)
            xs.append(xB); ys.append(yB)

        # 中心（黄色い星）— 円より背面
        self.ax.scatter([Ox], [Oy], s=200, marker="*", c="gold", edgecolors="k", linewidths=0.5, zorder=1)

        # 範囲はB全点にフィット（余白10%）
        if xs and ys:
            minx, maxx = min(xs), max(xs)
            miny, maxy = min(ys), max(ys)
            pad = 0.1 * max(maxx - minx + 1e-9, maxy - miny + 1e-9)
            self.ax.set_xlim(minx - pad, maxx + pad)
            self.ax.set_ylim(miny - pad, maxy + pad)

        self.ax.set_title("2D: B points (blue), center star (yellow)")
        self.canvas.draw()

    def update_plot_3d(self, params):
        """3D描画（A=赤，B=青，両方表示）"""
        self._switch_to_3d()
        Ox, Oy, Oz, lx, ly = params["Ox"], params["Oy"], params["Oz"], params["lx"], params["ly"]

        # BのXY枠（z=Oz）を参考表示
        bx = [Ox - lx/2, Ox + lx/2, Ox + lx/2, Ox - lx/2, Ox - lx/2]
        by = [Oy - ly/2, Oy - ly/2, Oy + ly/2, Oy + ly/2, Oy - ly/2]
        bz = [Oz]*5
        self.ax.plot(bx, by, bz, linestyle=":", linewidth=1)

        ptsB = self.points_3d["B"]
        ptsA = self.points_3d["A"]
        n = max(len(ptsB), len(ptsA), 1)

        # B: 青
        for pid, x, y, z in ptsB:
            alpha = 1.0 - 0.8 * ((pid - 1) / max(1, n - 1))
            self.ax.scatter([x], [y], [z], s=25, depthshade=True, c="tab:blue", alpha=alpha)

        # A: 赤
        for pid, x, y, z in ptsA:
            alpha = 1.0 - 0.8 * ((pid - 1) / max(1, n - 1))
            self.ax.scatter([x], [y], [z], s=25, depthshade=True, c="tab:red", alpha=alpha)

        # 中心（B平面上の中心に星）
        self.ax.scatter([Ox], [Oy], [Oz], s=200, marker="*", c="gold", edgecolors="k", linewidths=0.5)

        # 3Dの表示範囲をA/B全点でフィット（立方体比率寄せ）
        xs = [x for _, x, _, _ in ptsB] + [x for _, x, _, _ in ptsA]
        ys = [y for _, _, y, _ in ptsB] + [y for _, _, y, _ in ptsA]
        zs = [z for _, _, _, z in ptsB] + [z for _, _, _, z in ptsA]
        if xs and ys and zs:
            minx, maxx = min(xs), max(xs)
            miny, maxy = min(ys), max(ys)
            minz, maxz = min(zs), max(zs)
            rangex = maxx - minx
            rangey = maxy - miny
            rangez = maxz - minz
            max_range = max(rangex, rangey, rangez, 1.0)
            cx = 0.5 * (minx + maxx)
            cy = 0.5 * (miny + maxy)
            cz = 0.5 * (minz + maxz)
            half = 0.6 * max_range
            self.ax.set_xlim(cx - half, cx + half)
            self.ax.set_ylim(cy - half, cy + half)
            self.ax.set_zlim(cz - half, cz + half)

        self.ax.set_title("3D: A(red) & B(blue), center star (yellow)")
        self.canvas.draw()


if __name__ == "__main__":
    app = PushPlanGUI()
    app.mainloop()
