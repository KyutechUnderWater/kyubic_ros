# sbl_direction_estimator_node(音響班)が発行する、ハイドロフォンアレイから推定した
# ピンガー方向。kyubic_ws/src/sample/othersapmleCode/PingerDirection.msg
# (実装参考コード)のフィールド定義に準拠。

std_msgs/Header header

# 推定した方位 [deg]
float64 yaw
float64 pitch

# 最適化計算のスコア(誤差)。値が小さいほど良い(confidenceのような0-1のスケールではない)
float64 score
