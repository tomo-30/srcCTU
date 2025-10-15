# 走行フロー

```mermaid
flowchart LR
    GUI["① gui/app.py\n司令塔/状態機械"]
    MV["② map_viewer.py\n(読取専用で可視化)"]
    SR["③ scan_rough.py\n(粗)"]
    SP["④ scan_precise.py\n(精密)"]
    HV["⑤ harvest_sim.py\n(3秒完了)"]
    MB["⑥ move_base_sim.py\n(3秒移動)"]

    MS["map/map_state.json"]
    RQ["map/roi_queue.json"]
    DT["map/detections.json"]
    PC["pcl/results_pcl.jsonl"]

    GUI -- 起動/引数(JSON) --> SR
    GUI -- 起動/引数(JSON) --> SP
    GUI -- 起動/引数(JSON) --> HV
    GUI -- 起動/引数(JSON) --> MB
    GUI <-- stdout(JSON)/終了コード --- SR
    GUI <-- stdout(JSON)/終了コード --- SP
    GUI <-- stdout(JSON)/終了コード --- HV
    GUI <-- stdout(JSON)/終了コード --- MB

    SR -- 追記/更新 --> PC
    SR -- 更新 --> DT
    SR -- 更新(確率/訪問) --> MS

    SP -- 追記/更新 --> PC
    SP -- 候補追加 --> DT
    SP -- 更新(確信度↑) --> MS

    HV -- 収穫状態更新 --> MS

    MB -- 自己位置/軌跡更新 --> MS

    MV -- 読取 --> MS
    MV -- 読取 --> RQ
    MV -- 読取 --> DT
