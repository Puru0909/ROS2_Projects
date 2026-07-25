import threading
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory


class GesturePlayer:

    def __init__(self, node):
        self.node = node

        # Isolated node — completely separate from GestureManager's executor
        self.action_node = rclpy.create_node('gesture_player_client')

        self.client = ActionClient(
            self.action_node,
            FollowJointTrajectory,
            "/arm_controller/follow_joint_trajectory",
        )

        # Isolated executor spinning in its own thread permanently —
        # this is what avoids the deadlock we hit earlier
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.action_node)
        self._exec_thread = threading.Thread(
            target=self.executor.spin,
            daemon=True
        )
        self._exec_thread.start()

    def play(self, trajectory: JointTrajectory):
        if not self.client.wait_for_server(timeout_sec=5.0):
            self.node.get_logger().error("Action server not available")
            return False

        done_event = threading.Event()
        result_container = [None]
        timer_container = [None]

        def on_result(future):
            # Runs in isolated executor thread
            try:
                result_wrapper = future.result()
                status = result_wrapper.status
                error_code = result_wrapper.result.error_code
                error_string = result_wrapper.result.error_string

                if (status == GoalStatus.STATUS_SUCCEEDED
                        and error_code == FollowJointTrajectory.Result.SUCCESSFUL):
                    result_container[0] = True
                    self.node.get_logger().info(
                        "Trajectory execution completed successfully"
                    )
                else:
                    result_container[0] = False
                    self.node.get_logger().error(
                        f"Trajectory did not succeed — status={status}, "
                        f"error_code={error_code}, error_string='{error_string}'"
                    )
            except Exception as e:
                self.node.get_logger().error(f"Result error: {e}")
                result_container[0] = False
            finally:
                done_event.set()

        def on_goal_response(future):
            # Runs in isolated executor thread
            try:
                goal_handle = future.result()
                if not goal_handle.accepted:
                    self.node.get_logger().error("Trajectory rejected")
                    result_container[0] = False
                    done_event.set()
                    return
                result_future = goal_handle.get_result_async()
                result_future.add_done_callback(on_result)
            except Exception as e:
                self.node.get_logger().error(f"Goal response error: {e}")
                result_container[0] = False
                done_event.set()

        def submit_goal():
            # Runs in isolated executor thread via timer —
            # this is what let send_goal_async register with the right executor
            goal = FollowJointTrajectory.Goal()
            goal.trajectory = trajectory
            goal_future = self.client.send_goal_async(goal)
            goal_future.add_done_callback(on_goal_response)
            if timer_container[0] is not None:
                self.action_node.destroy_timer(timer_container[0])

        timer_container[0] = self.action_node.create_timer(0.01, submit_goal)

        done_event.wait(timeout=60.0)

        if result_container[0] is None:
            self.node.get_logger().error("Timeout: no result received")
            return False

        return result_container[0]
