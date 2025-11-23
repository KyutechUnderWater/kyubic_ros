import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from planner_msgs.action import FindPinger
import time

class DummyPingerServer(Node):
    def __init__(self):
        super().__init__('sbl_node') # 本物と同じノード名を名乗る
        self._action_server = ActionServer(
            self,
            FindPinger,
            'find_pinger', # 本物と同じアクション名
            self.execute_callback)
        self.get_logger().info('Dummy SBL (FindPinger) Server Started. Ready to fake!')

    def execute_callback(self, goal_handle):
        self.get_logger().info('Received Goal. Finding Pinger...')
        
        # フィードバックを適当に送る（探索してるフリ）
        feedback_msg = FindPinger.Feedback()
        for i in range(3):
            feedback_msg.current_yaw = float(i * 10.0)
            feedback_msg.current_score = 0.5 + (i * 0.1)
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f'Feedback sent: Yaw={feedback_msg.current_yaw}')
            time.sleep(1.0) # 1秒待つ

        # 成功として終了
        goal_handle.succeed()
        result = FindPinger.Result()
        result.success = True
        result.final_yaw = 30.0
        self.get_logger().info('Goal Succeeded. Pinger found!')
        return result

def main(args=None):
    rclpy.init(args=args)
    node = DummyPingerServer()
    rclpy.spin(node)

if __name__ == '__main__':
    main()