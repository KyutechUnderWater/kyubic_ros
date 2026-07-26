# ハイドロフォン測角のスタブ用インターフェース。
# 実装なし(未実装)。実装時はこのメッセージを実データで発行するノードに差し替える。
std_msgs/Header header

# 仰角/俯角 [deg]。正 = 対象が下方向(body z正方向、NED慣習に合わせる)
float32 pitch_deg

# 水平面内の方位角 [deg]。前方(body x正方向)を0度として反時計回りが正
float32 yaw_deg
