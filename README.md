# aichallenge-tools

Japan Automotive AI Challenge 用の自作開発・解析ツール集です。

このリポジトリは公式の `aichallenge-racingkart` とは独立しています。
公式環境を変更せず、既に導入済みの AI Challenge 環境に対して外部からツールを実行する想定です。

## 前提

利用するPCに、公式の AI Challenge 環境が導入済みであることを前提としています。

標準では次の場所を使用します。

```text
~/aichallenge-racingkart
```

また、Docker / Docker Compose が利用でき、通常どおり

```bash
cd ~/aichallenge-racingkart
make dev
```

でシミュレータを起動できる状態にしてください。

---

## 導入方法

GitHub アカウントは不要です。公開リポジトリなので、そのまま clone できます。

```bash
cd ~
git clone https://github.com/saa-waa0050/aichallenge-tools.git
cd ~/aichallenge-tools
```

既に clone 済みの場合は、更新時に以下を実行します。

```bash
cd ~/aichallenge-tools
git pull
```

> リポジトリ名を変更している場合は、上記URLを実際のURLに読み替えてください。

---

# Race Telemetry

`race_telemetry` は、走行中のROS 2データを記録し、走行後にCSVとHTMLレポートを生成する解析ツールです。

主に以下を確認できます。

- 32 km/h を基準とした速度分布
- 加速 / 減速
- ステアリング角
- ブレーキ指令
- 走行軌跡上のヒートマップ
- 距離に対する速度・加速度・操舵・ブレーキのグラフ
- 基準速度に対する概算タイムロス

## 使い方

### 1. テレメトリを待機させる

ターミナル1で実行します。

```bash
cd ~/aichallenge-tools
bash race_telemetry/run.sh
```

正常なら、Autowareコンテナが起動するまで待機します。

```text
Waiting for the autoware container...
```

この状態のままにしてください。

### 2. AI Challenge を通常どおり起動する

別のターミナルを開きます。

```bash
cd ~/aichallenge-racingkart
make dev
```

`autoware` コンテナが起動すると、`race_telemetry` が自動で検出して記録を開始します。

シミュレータ側は普段どおり操作してください。

### 3. 走行終了後にレポートを保存する

走行が終わったら、`race_telemetry` を実行しているターミナルで

```text
Ctrl+C
```

を押します。

CSVとHTMLレポートが次の場所へ保存されます。

```text
~/aichallenge-tools/race_telemetry/runs/
└── run_YYYYMMDD_HHMMSS/
    ├── telemetry.csv
    └── telemetry.html
```

`telemetry.html` をブラウザで開くと解析結果を確認できます。

---

## AI Challenge の配置場所が違う場合

`aichallenge-racingkart` が `~/aichallenge-racingkart` 以外にある場合は、パスを引数で指定できます。

```bash
cd ~/aichallenge-tools
bash race_telemetry/run.sh /path/to/aichallenge-racingkart
```

例:

```bash
bash race_telemetry/run.sh ~/work/aichallenge-racingkart
```

環境変数でも指定できます。

```bash
AICHALLENGE_REPO=~/work/aichallenge-racingkart \
bash race_telemetry/run.sh
```

---

## 基準速度を変更する

デフォルトは `32 km/h` です。

例えば `35 km/h` を基準にする場合:

```bash
TELEMETRY_BASE_SPEED=35 bash race_telemetry/run.sh
```

---

## サンプリング周波数を変更する

デフォルトは `20 Hz` です。

例えば `40 Hz` にする場合:

```bash
TELEMETRY_SAMPLE_HZ=40 bash race_telemetry/run.sh
```

---

## Race Telemetry が使用するROS 2トピック

- `/localization/kinematic_state`
- `/vehicle/status/velocity_status`
- `/vehicle/status/steering_status`
- `/control/command/actuation_cmd`

公式環境の制御内容やシミュレータ設定を書き換えるツールではありません。

---

## ディレクトリ構成

```text
aichallenge-tools/
├── README.md
└── race_telemetry/
    ├── README.md
    ├── race_telemetry.py
    ├── run.sh
    └── runs/              # 実行後に生成
```

`runs/` は `.gitignore` の対象なので、解析結果は通常GitHubにはアップロードされません。
