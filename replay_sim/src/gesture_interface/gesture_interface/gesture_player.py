import threading
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory


class GesturePlayer:

    def __init__(self, node):
        self.node = node

        # Separate node — completely isolated from GestureManager executor
        self.action_node = rclpy.create_node('gesture_player_action')

        self.client = ActionClient(
            self.action_node,
            FollowJointTrajectory,
            "/arm_controller/follow_joint_trajectory",
        )

        # Dedicated executor — runs in its own thread
        # Completely independent from GestureManager executor
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.action_node)
        self.executor_thread = threading.Thread(
            target=self.executor.spin,
            daemon=True
        )
        self.executor_thread.start()

    def play(self, trajectory: JointTrajectory):
        if not self.client.wait_for_server(timeout_sec=5.0):
            self.node.get_logger().error("Controller action server not available")
            return False

        done_event = threading.Event()
        result_container = [None]

        def on_result(future):
            try:
                future.result()
                result_container[0] = True
                self.node.get_logger().info("Trajectory execution completed")
            except Exception as e:
                self.node.get_logger().error(f"Result error: {e}")
                result_container[0] = False
            finally:
                done_event.set()

        def on_goal_response(future):
            try:
                goal_handle = future.result()
                if not goal_handle.accepted:
                    self.node.get_logger().error("Trajectory rejected")
                    result_container[0] = False
                    done_event.set()
                    return
                # get_result_async runs in separate executor thread — no deadlock
                result_future = goal_handle.get_result_async()
                result_future.add_done_callback(on_result)
            except Exception as e:
                self.node.get_logger().error(f"Goal response error: {e}")
                result_container[0] = False
                done_event.set()

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory

        # Processed by separate executor thread — not GestureManager thread
        future = self.client.send_goal_async(goal)
        future.add_done_callback(on_goal_response)

        # GestureManager service thread waits here
        # Separate executor handles all action callbacks freely
        done_event.wait(timeout=60.0)

        if result_container[0] is None:
            self.node.get_logger().error("Timeout: no result received")
            return False

        return result_container[0]
