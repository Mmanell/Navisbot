#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Quaternion
from rclpy.action import ActionClient
import math, time

def pose_from_xyyaw(x, y, yaw, frame='map'):
    ps = PoseStamped()
    ps.header.frame_id = frame
    ps.header.stamp = rclpy.time.Time().to_msg()
    ps.pose.position.x = x
    ps.pose.position.y = y
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)
    ps.pose.orientation = Quaternion(x=0.0, y=0.0, z=qz, w=qw)
    return ps

class PlannerBenchmarkNode(Node):
    def __init__(self):
        super().__init__('planner_benchmark')
        self._action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.get_logger().info("Waiting for /navigate_to_pose action server...")
        self._action_client.wait_for_server()
        self.get_logger().info("Action server available.")

    def send_goal(self, goal_pose):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose

        t0 = time.time()
        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            raise RuntimeError("Goal rejected by server")

        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)
        t1 = time.time()

        elapsed = t1 - t0
        return elapsed

    def feedback_callback(self, feedback_msg):
        # Optional: print intermediate feedback
        pass

def main():
    rclpy.init()
    node = PlannerBenchmarkNode()
    goal_pose  = pose_from_xyyaw(2.0, 2.0, 0.0)

    node.get_logger().info(f"Benchmarking goal: ({goal_pose.pose.position.x},{goal_pose.pose.position.y})")
    try:
        plan_time = node.send_goal(goal_pose)
        node.get_logger().info(f"Planning + execution time: {plan_time:.3f} s")
    except Exception as e:
        node.get_logger().error(f"Failed: {e}")

    rclpy.shutdown()

if __name__ == "__main__":
    main()
