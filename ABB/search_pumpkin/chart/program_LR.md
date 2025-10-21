# 走行フロー

```mermaid
flowchart LR
  subgraph Inputs[入力ソース]
    PLAN[info push_plan json]
    PINS[Arduino 9ピン シリアル]
    GUI[GUI 入力 速度 ゾーン 閾値 ログモード]
  end

  subgraph Core[実行コア main_py]
    MAIN[main main_py 実行ループ]
    PM[PipelineMover 先行送信 差分Speed Zone]
    ZS[ZoneSpeedCache 差分送信]
  end

  subgraph RobotPath[ロボット経路]
    A1[A_i 退避点]
    B1[B_i 押し込み点]
  end

  subgraph Robot[ロボット側]
    RC[RobotClient]
    OA[open_abb]
    RAPID[RAPID SERVER mod]
  end

  subgraph Outputs[出力]
    LOG[log イベント json]
    VIEW[GUI ピン表示 現在姿勢]
  end

  %% 入力
  PLAN --> MAIN
  PINS --> MAIN
  GUI --> MAIN

  %% 計画展開と移動
  MAIN --> PM
  MAIN --> ZS
  MAIN --> A1
  MAIN --> B1
  PM --> RC
  ZS --> RC

  %% ロボット通信スタック
  RC --> OA
  OA --> RAPID

  %% 取得データとログ
  RC --> MAIN
  MAIN --> LOG
  MAIN --> VIEW
