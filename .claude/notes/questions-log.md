## Q: kyubic_ws の実行方法
- 状況: リポジトリ全体 / kyubic_ws（ROS 2 Jazzy ワークスペース）
- 期待 vs 実際: 実行手順がわからない（初回セットアップ〜起動まで）
- 回答要約: Docker で `ros2_start` → コンテナ内で `build`（colcon）→ `byobu` または `ros2 launch kyubic_bringup kyubic.launch.py` 等で起動。初回は `install.sh` と submodule 取得が必要。
- 日付: 2026-07-24

## Q: build と colcon は何のため？
- 状況: コンテナ内の `build` エイリアス / `colcon build`
- 期待 vs 実際: 実行前に build が必要な理由と colcon の意味がわからない
- 回答要約: ソース（.cpp/.py）を `ros2 launch` で動かせる実行ファイル・メッセージ型に変換する工程。colcon はワークスペース全体のビルドツール。`build` はその短縮形。コードを変えたら再 build が必要。
- 日付: 2026-07-24

## Q: setup.bash・ビルドファイル・launch 順序の意味
- 状況: `install/setup.bash`、`CMakeLists.txt`、`mavlink_driver/setup.py`、byobu の3 launch
- 期待 vs 実際: 「install」が何か、ビルドファイルの役割、launch の起動順がわからない
- 回答要約: setup.bash は install/ 成果物をシェルに登録（apt とは別）。CMakeLists.txt/setup.py は言語別ビルド設計書。launch 順は論理依存（ドライバ→推定→行動ツリー）だが ROS は強制しない。
- 日付: 2026-07-24