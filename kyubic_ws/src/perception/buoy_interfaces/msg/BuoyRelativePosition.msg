# ブイ検出結果、画面中心からの正規化偏差、ヨー角、および機体相対位置。
# header.stamp: 入力画像の撮影時刻。
# header.frame_id: 座標値とヨー角が属する機体座標系。既定値は"base_link"。
# このシステムのbase_linkは X:前、Y:右、Z:下（FRD）とする。
std_msgs/Header header

# YOLOが対象クラスのBBoxを検出した場合にtrue。
bool detected

# ヨー角を利用できる場合にtrue。
# 位置推定を無効化していても、BBoxとカメラ内部パラメータが有効ならtrueになる。
bool yaw_valid

# X・Y・Zの相対座標を利用できる場合にtrue。
bool position_valid

# 採用したBBoxのYOLO信頼度。未検出時は0.0。
float32 confidence

# BBox中心の、画像中心からの符号付き水平偏差を画像半幅で正規化した値。
# おおむね -1.0:左端、0.0:中央、+1.0:右端。右方向を正とする。
# カメラ内部パラメータや対象距離には依存しない。未検出時はNaN。
float32 normalized_horizontal_offset

# BBox中心の、画像中心からの符号付き垂直偏差を画像半高さで正規化した値。
# おおむね -1.0:上端、0.0:中央、+1.0:下端。下方向を正とする。
# カメラ内部パラメータや対象距離には依存しない。未検出時はNaN。
float32 normalized_vertical_offset

# 機体正面からブイ方向への水平ヨー角[rad]。
# base_link FRDにおいて右旋回方向を正、左旋回方向を負とする。
# yaw_valid=falseの場合はNaN。
float32 relative_buoy_yaw_rad

# header.frame_idで示す機体座標系における相対位置[m]。
# X: 機体前方、Y: 機体右方向、Z: 機体下方向。
# position_valid=falseの場合はNaN。
float32 relative_buoy_x_m
float32 relative_buoy_y_m
float32 relative_buoy_z_m
