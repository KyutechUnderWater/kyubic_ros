# Tools

開発時のROS 2 bag解析、ログ可視化、および実験用スクリプトを置くディレクトリです。
ROSパッケージとしてビルド・インストールされるものではありません。ROS 2 Jazzyの環境を読み込んでから実行してください。

## ROS 2 bagの解析

bag形式はMCAPを優先し、開けない場合はSQLite 3を試します。

| スクリプト | 用途 | 実行例 |
| --- | --- | --- |
| `extract_depth_csv.py` | `/localization/depth_odom` から深度をCSVへ抽出 | `python3 tools/extract_depth_csv.py <bag_path> --output tools/depth_data.csv` |
| `extract_gnss_csv.py` | `/localization/gnss/odom` から緯度・経度をCSVへ抽出 | `python3 tools/extract_gnss_csv.py <bag_path> --output tools/gnss_data.csv` |
| `plot_force_gnss.py` | GNSS位置と推進力をPNGとして描画 | `python3 tools/plot_force_gnss.py <bag_path> tools/force_gnss_plot.png` |
| `plot_depth_data.py` | `depth_data.csv` を深度グラフとして描画 | `python3 tools/plot_depth_data.py` |
| `plot_gnss_data.py` | `gnss_data.csv` を緯度・経度の軌跡として描画 | `python3 tools/plot_gnss_data.py` |

`plot_depth_data.py` と `plot_gnss_data.py` の既定入出力先は、この `tools` ディレクトリです。必要に応じて `plot_gnss_data.py` の `--input` と `--output` を指定できます。

## Jetson向けウニ検出ノード

`jetson_urchin_tracker.py` は、ZEDカメラのRGB画像とDepth画像からYOLOでウニを検出し、`/urchin_target_position` へ3次元座標をPublishするROS 2ノードです。

実行には、ROS 2のほか `ultralytics`、OpenCV、NumPy、`cv_bridge`、`message_filters` が必要です。YOLOモデルはROSパラメータ `yolo_model` で指定できます。

```bash
python3 tools/jetson_urchin_tracker.py --ros-args -p yolo_model:=/path/to/model.pt
```

使用トピックはスクリプト内で設定しています。実機のZED構成に合わせて確認してください。

## 生成済みデータ

`*_data.csv` と `*_plot.png` は、上記ツールで利用・生成するサンプルまたは解析結果です。再生成する場合は、対応する抽出・可視化スクリプトを使用してください。

## scratch

`scratch/` は検証・調査用の一時的な補助コードです。`plot_bag.py` と `print_values.py` は対象bagパスやトピックをスクリプト内で指定するため、実行前に環境に合わせて編集してください。`test_conv.cpp` と `test_conv` は座標変換の簡易検証用です。
