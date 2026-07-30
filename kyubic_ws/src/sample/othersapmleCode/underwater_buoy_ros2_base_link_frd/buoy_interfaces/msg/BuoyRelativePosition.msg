# ブイ検出結果と機体座標系における相対位置。
# header.stamp: 入力画像の撮影時刻。
# header.frame_id: 座標値が属する機体座標系。既定値は"base_link"。
# このシステムのbase_linkは X:前、Y:右、Z:下（FRD）とする。
std_msgs/Header header

# YOLOが対象クラスのBBoxを検出した場合にtrue。
bool detected

# X・Y・Zの相対座標を利用できる場合にtrue。
bool position_valid

# 採用したBBoxのYOLO信頼度。未検出時は0.0。
float32 confidence

# header.frame_idで示す機体座標系における相対位置[m]。
# X: 機体前方、Y: 機体右方向、Z: 機体下方向。
# position_valid=falseの場合はNaN。
float32 relative_buoy_x_m
float32 relative_buoy_y_m
float32 relative_buoy_z_m
