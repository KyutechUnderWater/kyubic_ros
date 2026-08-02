# 別ラズパイのsbl_control等が発行する、ピンガー推定位置。
# localization/odom と同じ NED 座標系 (x/y 水平、z_depth 正=下) を想定。

std_msgs/Header header

# 推定位置 [m]
float64 x
float64 y
float64 z_depth

# 最適化スコア(誤差)。PingerDirection と同様、小さいほど良い
float64 score
