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

## トラブルシューティング

### エラー：`AllocateTensors() failedERR: Failed to run classifier (-3)`が出る場合

#### 原因

ESP32-S3のハードウェア演算最適化（ESP-NN）を利用して推論を行う場合、計算を高速化するための「スクラッチバッファ（作業用メモリ）」が追加で必要になります。この要求サイズは、Edge Impulseがエクスポート時にデフォルトで見積もる「Tensor Arena」のサイズを大幅に上回るため、メモリ確保エラーが発生します（例：エクスポート時には21683バイトだったのに実際に必要としたのは84287バイト）。

#### 解決策

エクスポートしたライブラリ内の2つのヘッダファイルを修正し、確保するメモリのサイズを手動で増やします。

#### 手順1：システム全体の上限サイズを増やす

- ファイル：`src/model-parameters/model_metadata.h`
- `EI_CLASSIFIER_TFLITE_LARGEST_ARENA_SIZE`の値を、余裕を持った数値（例：`90000`〜`150000`）に変更します。

```c
// 変更前（数値はモデルにより異なります）
#define EI_CLASSIFIER_TFLITE_LARGEST_ARENA_SIZE     21683

// 変更後（約90KB〜150KBなど、十分な余裕を持たせる）
#define EI_CLASSIFIER_TFLITE_LARGEST_ARENA_SIZE     90000
```

#### 手順2：モデルへの実際の割り当てサイズを増やす

- ファイル：`src/model-parameters/model_variables.h`
- 特定のモデルに割り当てる`.arena_size`の末尾に、追加のマージン（例：`+ 65000`など）を追記します。

```c
// 変更前
const ei_config_tflite_graph_t ei_config_graph_... = {
    // ...
    .arena_size = tflite_learn_..._arena_size
};

// 変更後（+ 65000 などを追記してバッファを増やす）
const ei_config_tflite_graph_t ei_config_graph_... = {
    // ...
    .arena_size = tflite_learn_..._arena_size + 65000
};
```

#### 手順3：強制的に再コンパイルする（重要）

Arduino IDEはライブラリの変更をキャッシュしてしまう仕様があります。変更を確実に反映させるため、メインの`.ino`スケッチファイルに空行を1行追加など変更をくわえて上書き保存してから書き込み（コンパイル）を実行してください。

#### 【高度な設定】実際のメモリ使用量（Arena Size）を正確に計測する方法

ESP32-S3のハードウェア最適化（ESP-NN）などが要求する正確なメモリ量は、実際にモデルを動かしてみるまで分かりません。極限までマイコンのRAMを節約したい場合は、推論エンジンが実際に消費したバイト数を出力させ、それに「+2KB（2048バイト）程度」の余裕を持たせた値を`model_metadata.h`と`model_variables.h`に設定するのが最も効率的です。

変更するファイルは`src/edge-impulse-sdk/classifier/inferencing_engines/tflite_micro.h`です（注：Edge Impulse側で「EON Compiler」を有効にしてエクスポートした場合は`tflite_eon.h`を修正してください）。

ファイル内で `AllocateTensors()` を検索し、メモリ確保の成功を判定しているブロックを抜けた直後に、実際の消費バイト数を出力する `ei_printf` を追加します。

```cpp
        // 変更前（既存のコード）
        TfLiteStatus allocate_status = interpreter->AllocateTensors();
        if (allocate_status != kTfLiteOk) {
            ei_printf("ERR: Failed to allocate TFLite arena (error code %d)\n", allocate_status);
            return EI_IMPULSE_TFLITE_ARENA_ALLOC_FAILED;
        }

        // 変更後（下記2行を追加）
        ei_printf("DEBUG: TFLite Arena used bytes: %d\n", 
                  (int)interpreter->arena_used_bytes());
```

メインの`.ino`ファイルを上書き保存して再コンパイルすると、最初の推論時にシリアルモニタへ`DEBUG: TFLite Arena used bytes: 84287`のように正確な消費量が出力されるようになります。確認後はこの追加コードを削除（またはコメントアウト）してください。
