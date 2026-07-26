# 音響(SBL)によるピンガー方向推定のスタブ用インターフェース。
# 実装なし(未実装)。実装時は sbl_estimation_node がこのメッセージを実データで発行する。
std_msgs/Header header

# 有効な方向推定が得られているか(BuoyDetection.detectedと同じ役割)
bool detected

# 水平面内の方位角 [deg]。前方(body x正方向)を0度として反時計回りが正
float32 yaw_deg

# 仰角/俯角 [deg]。正 = 対象が下方向(body z正方向、NED慣習に合わせる)
float32 pitch_deg

# 推定の信頼度スコア。スケールは未確定(音響班と要すり合わせ)のため、
# bluerov_control側のhydrophone_score_threshold既定値は0.0(常に許可)としている
float32 score
