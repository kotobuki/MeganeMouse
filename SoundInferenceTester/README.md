# MeganeMouse Sound Inference Tester

MeganeMouse本体に音声クリック機能を組み込む前に、Edge Impulseで学習させた推論モデルの動作検証・パフォーマンス評価を行うためのテストツールです。

マイクから取得した音声を連続的に推論（Continuous Inference）し、各クラスの確率や処理時間（DSPおよび推論にかかった時間）をシリアルモニタに詳細に出力します。

## 特徴

- デュアルコア（FreeRTOS）設計：Core 0の専用タスクで音声キャプチャと機械学習の推論を実行し、Core 1をメインループ（UI描画や今後のBLE/マウス制御）のために完全に開放しています。
- リアルタイムモニタリング：シリアルモニタを通じて、クリック音の検出確率やノイズの判定状況をリアルタイムで確認できます。

## ハードウェア要件

- M5Stack AtomS3R
- USB-Cケーブル（データ通信対応のもの）
- マイク（以下のいずれかを使用）：
  - PDMマイク（例：Unit Mini PDM等をPort Aに接続）
  - M5Stack Atomic Echo Base（AtomS3Rの底面に装着）

## セットアップ

### 1. ライブラリの準備

1. Edge Impulseのダッシュボードから、学習済みモデルを「Arduino Library」としてビルドし、ダウンロードします。
2. Arduino IDEのメニューから`スケッチ` > `ライブラリをインクルード` > `.ZIP形式のライブラリをインストール`を選択し、ダウンロードしたZIPファイルを追加します。
3. `SoundInferenceTester.ino`を開き、インクルードしているヘッダファイル名（例：`<sound-click_inferencing.h>`）をご自身のモデル名に合わせて変更してください。

### 2. 【重要】レスポンスの向上（スライス数の変更）

クリック音のような極めて短い突発音の検出漏れを防ぎ、レスポンス（遅延）を劇的に向上させるため、推論の「窓（スライス）」を細かく分割することを強く推奨します。

注意：この変更は`.ino`ファイル内の`#define`では行えません。コンパイル時のメモリ破壊を防ぐため、必ずEdge Impulseライブラリ側のソースコードを直接編集してください。

1. Arduinoのライブラリフォルダ（例：`Documents/Arduino/libraries/対象のモデル名/`）を開きます。
2. `src/model-parameters/model_metadata.h`をテキストエディタで開きます。
3. 以下の行を探し、数値を`4`から`8`に変更して保存します。

   ```c
   #define EI_CLASSIFIER_SLICES_PER_MODEL_WINDOW    8
   ```

（※Edge Impulseからライブラリを再度ダウンロードして上書きした場合は、この設定がリセットされるため再設定が必要です。）

### 3. ハードウェアの設定と書き込み

コード上部の`USE_PDM_MIC`の値を、使用するマイクに合わせて設定（PDMなら`1`、Echo Baseなら`0`）し、AtomS3Rに書き込みます。

## シリアルモニタの見方

115200 bpsでシリアルモニタを開くと、以下のようなログが連続して出力されます。

```text
[noise:100% click:0% z_openset:0%] (DSP:5ms Cls:2ms)
[noise:0% click:100% z_openset:0%] (DSP:5ms Cls:2ms)
Loop recvd click: click (100%)
```

- `[ ... ]`内の数値は、その時点での各クラスの確率（0〜100%）です。
- `(DSP:5ms Cls:2ms)`は、音声データの特徴量抽出（DSP処理）にかかった時間と、分類（推論）にかかった時間をそれぞれ示します。
- `Loop recvd click:`は、Core 0での推論結果が閾値を超え、Core 1のメインループにクリックイベントとして無事に伝達されたことを示しています。
