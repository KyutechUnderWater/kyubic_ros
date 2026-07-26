# sbl_interfaces/msg/PingerDirection.msg

# ROS2標準のヘッダー（タイムスタンプとフレームIDを含む）
std_msgs/Header header

# 推定した方位 [deg]
float64 yaw
float64 pitch

# 最適化計算のスコア（誤差）
float64 score