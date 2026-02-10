# ROS2 Jazzy Update Plan

このドキュメントは、`cabot-ros2-jazzy`をROS2 Jazzyへ更新するための「コードを書き始める前にやること」を中心にまとめた作業計画です。
現状、このリポジトリはROS2 Humble前提の記述/構成が多い（`humble`/`jammy-humble` 等）ため、まずは影響範囲の棚卸しと作業環境の確定を行います。

## 背景（現状）

- ホスト環境:
  - Ubuntu 20.04 + ROS2 Galactic
  - Ubuntu 22.04 + ROS2 Humble
- Docker環境:
  - Ubuntu 22.04 + ROS2 Humble（主に`jammy-humble`前提のベース/タグ）
- アーキテクチャ:
  - 開発環境: `amd64`
  - 実機: `arm64`
- 依存/カスタマイズ:
  - `navigation2`はHumble相当をベースにしたフォーク（`cmu-cabot/navigation2`の`humble-custom`）をソースビルドしている
  - 他にも`dependency*.repos`や`cabot-base/docker/humble-custom/**`配下にフォーク/パッチが多数あるため、Jazzy移行前に影響範囲の棚卸しが必須

## 目標（ターゲット）

- ホスト環境: Ubuntu 24.04（noble）+ ROS2 Jazzy
- Docker環境: Ubuntu 24.04（noble）+ ROS2 Jazzy
- アーキテクチャ: `amd64`（開発）/ `arm64`（実機）の両方でビルド・実行できる状態にする

## ブランチ方針

- 作業ブランチは`ros2-dev`から分岐する
- ブランチ名は `ros2-dev-<hoge>` 形式
  - 例: `ros2-dev-jazzy`, `ros2-dev-jazzy-migration`

## 着手前チェックリスト（コードを書き始める前）

1. 作業開始点の固定
   - `ros2-dev`が最新であることを確認し、作業開始コミットSHAを記録する
   - 作業前に`git status`がクリーンであることを確認する

1. 対象範囲の確定（ここが曖昧だと後工程が破綻する）
   - ホスト: Ubuntu 24.04（noble）+ Jazzy
   - Docker: Ubuntu 24.04（noble）+ Jazzy
   - アーキテクチャ: `amd64`/`arm64`
   - Jetson等の特殊環境がある場合:
     - 「実機OS自体を24.04へ上げる」のか、「ホストは据え置きでコンテナ/ワークスペースのみJazzy化する」のかを最初に決める
     - Dockerのベース（CUDA/JetPack等）制約で24.04が難しいケースがあるため、代替案（別系統のイメージ/段階移行）を用意する

1. 影響範囲の棚卸し（Humble前提の箇所を洗い出す）
   - 文字列検索で`humble`/`jammy-humble`/`ROS_DISTRO`参照箇所を一覧化する
   - `ros-humble-*`のようにディストロがハードコードされている箇所も対象に含める（`ros-$ROS_DISTRO-*`へ寄せる、など）
   - 特に以下のカテゴリを優先して洗う
     - Docker: `docker/**`, `docker-compose*.yaml`, `.env`で使うイメージ名/タグ
     - Docker bake: `docker-bake.hcl`（デフォルト`ROS_DISTRO`、contextパス、タグ命名）
     - ツール: `tools/install-host-ros2.sh` 等のインストーラ/セットアップ
     - 依存: `dependency*.repos`, `setup-dependency.sh`の挙動
     - ドキュメント: `README.md`, `doc/**`, `docker/README.md`

1. カスタム/フォーク依存の棚卸し（最優先）
   - `dependency*.repos`（ルート＋各サブリポジトリ配下）から「フォーク/独自ブランチ/固定コミット」を抽出し一覧化する
   - 既知の要注意例（現状の`.repos`より）:
     - `navigation2`: `https://github.com/cmu-cabot/navigation2` `humble-custom`（Jazzy相当へポーティングが必要）
     - `gazebo_ros_pkgs`: `https://github.com/CMU-cabot/gazebo_ros_pkgs` `humble-dev`
     - `realsense_gazebo_plugin`: `https://github.com/cmu-cabot/realsense_gazebo_plugin.git` `humble-devel`
     - `ros2_persist_parameter_server`: `double-value-workaround`
     - `cartographer`/`cartographer_ros`: `cabot-humble`
     - `bluespace_ai_xsens_ros_mti_driver`: `ros2_0_galactic`
     - `ublox`: `ros2-cabot`
     - `ntrip_client`: `feature/ros2_launch_argument`
     - `rosbridge_suite`: `ros2-dev`
     - `ros_odrive`: `cabot`
     - LiDAR/センサー系（`cabot-base/docker/humble-custom/**`配下）: Velodyne / Hesai / Livox / RoboSense / LS LiDAR など
     - people/realsense系（`cabot-people/**`配下）: `realsense_ros`固定バージョン、`jetson-humble-custom`ディレクトリの存在
   - 各カスタムについて以下を決める
     - Jazzyでapt提供されるためフォーク不要（置き換え）/ ソースビルド継続 / upstreamへ寄せる（差分削減）

1. 既知の移行リスク/要確認ポイント（早期に潰す）
   - `arm64`向けの暫定対応:
     - `cabot-base/docker/humble-custom/Dockerfile`に`arm64`限定のaptハック（OpenCV downgrade、Gazebo PPA追加）があるため、Ubuntu 24.04 + Jazzyで要再設計
   - `galactic`/`humble`に依存したブランチ:
     - `cabot-navigation/dependency.repos`に`ros2_0_galactic`等が残っているため、Jazzy対応ブランチ/代替の有無を確認
   - Dockerfileのディストロハードコード:
     - `ros-humble-*`のような直書きがある場合、Jazzy化の障害になりやすい（変数化/分離が必要）

1. 依存関係の更新戦略を決める（multi-repo前提）
   - 本リポジトリは`dependency.repos`でサブリポジトリを取り込む構成
     - 例: `cabot-base`, `cabot-navigation`, `cabot-drivers`, `cabot-people` など
   - Jazzy対応は複数リポジトリに跨る可能性が高いため、以下を先に決める
     - サブリポジトリ側にJazzy用ブランチを作るか（例: `jazzy`, `ros2-jazzy`）
     - どのブランチ/タグを取り込むか（固定する時期と、固定方法）

1. Nav2移行の進め方を決める（カスタム差分が大きい前提）
   - 現在の`cmu-cabot/navigation2`（`humble-custom`）の差分を抽出する
     - upstream（`ros-planning/navigation2`）との差分、フォーク内の独自コミット群、パッチ内容の分類
   - Jazzy側のNav2へ差分をどう反映するかを決める
     - (A) JazzyのNav2へcherry-pick/port
     - (B) パッチが不要なら削除
     - (C) Jazzyで破綻する場合は設計変更（Nav2以外含む）

1. ビルド/動作確認の「最小成功条件」を先に定義する
   - 例（後で具体化する）:
     - 主要コンテナがビルドできる
     - 主要launchが起動する（navigation/people/localization等）
     - 主要な静的解析・テストが通る（`colcon test`、lint、など）
   - 何をもって「Jazzy化完了」とするか（範囲外を明確化）もここで決める

## setup-dependency.sh の override（-o）挙動メモ

`setup-dependency.sh`は、依存リポジトリを`vcstool`で取り込むためのスクリプト。

- デフォルト（`-d`なし）:
  - `dependency-release.repos`が存在する場合、それを使って`vcs import`して終了（固定版）
- 開発用途（`-d`あり）:
  - ツリー内の`dependency.repos`を探索して順に`vcs import`する（分岐/循環参照を想定）
- override（`-o`）:
  - `dependency.repos`を読み込む際、同階層の`dependency-override.repos`があればそれを追記してimportする
  - 実装上は「`dependency-override.repos`から`repositories:`行を除去して一時ファイルに追記」している
  - ルートだけでなく、サブリポジトリ配下の`dependency.repos`にも同様に適用される

Jazzy移行時の推奨運用:

- 作業中は固定版ではなくブランチ追従で回すため、基本は `./setup-dependency.sh -d -o` を使う
- Jazzy対応のために一時的に特定リポジトリ/ブランチへ切り替える場合は、`dependency-override.repos`に追記して全員の作業環境を揃える
  - ただし、このファイルはコミット可能な一方でマージコンフリクトを生みやすいので、運用ルール（誰がいつ更新するか）を決める

## 作業フェーズ（着手後の大枠）

1. ベース更新（Docker/OS/ROS_DISTROの整合）
   - `humble`前提のDockerコンテキスト/イメージ名/compose設定をJazzy向けに整理
   - `tools/install-host-ros2.sh`等の`ROS_DISTRO`分岐をJazzy対応に更新

1. 依存関係更新（repos/apt/rosdep）
   - Jazzyで入手可能な依存（apt）と、ソースビルドが必要な依存を切り分け
   - 必要に応じてサブリポジトリのブランチを切り、`dependency-override.repos`で追従

1. ビルドを通す（最小構成から）
   - まずは`colcon build`が通る最小ワークスペースを作る
   - その後、機能（navigation/people/localization等）を段階的に戻す

1. 動作確認と固定（release化）
   - 成功条件に沿ってテスト/起動確認
   - 固定版が必要になったタイミングで`dependency-release.repos`を更新（`setup-dependency.sh -r`）
