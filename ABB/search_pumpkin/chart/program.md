# meinフロー

```mermaid
flowchart TD
  A[起動: Tk GUI構築] --> B[自動接続: ロボット接続と計画読込]
  A --> C[Arduino監視開始]
  A --> D[GUIタイマー開始]

  B --> E{Start押下?}
  C --> E
  D --> E

  E -->|はい| F[ログモード決定]
  F --> G[パラメータ取得]
  G --> H[_run_plan_loop開始]

  H --> I{残りターゲットあり?}
  I -->|いいえ| Z[完了]
  I -->|はい| J[次ターゲット取得 label pose id]
  J --> K[PipelineMover準備 差分Speed Zone 先行送信]
  K --> L{Bターゲットか?}

  L -->|はい| M[中断条件セット ピン閾値 一時停止 スキップ]
  L -->|いいえ| N[通常移動]

  M --> O[MoveLパイプライン実行]
  N --> O

  O --> P{中断発生?}
  P -->|一時停止またはスキップ| Q[中断ログ user_abort]
  P -->|ピン閾値②③発火| R[停止 待機 pins取得 ログ pin_trigger スキップ]
  P -->|なし| S[到達処理 ログ reached]

  S --> T{最終点かつFine設定あり?}
  T -->|はい| U[ZoneをFineに変更 MoveL_ack Zone復帰]
  T -->|いいえ| V[次ターゲットへ]

  U --> V
  Q --> V
  R --> V
  V --> I
