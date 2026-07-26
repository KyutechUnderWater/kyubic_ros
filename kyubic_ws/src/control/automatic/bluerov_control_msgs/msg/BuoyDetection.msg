# 画像処理によるブイ検出のスタブ用インターフェース。
# 実装なし(未実装)。実装時はこのメッセージを実データで発行するノードに差し替える。
std_msgs/Header header

bool detected

# 機体座標系(body frame)での相対位置 [m]
float32 relative_buoy_x_m
float32 relative_buoy_y_m
float32 relative_buoy_z_m

# 検出信頼度 [0.0-1.0]
float32 confidence
