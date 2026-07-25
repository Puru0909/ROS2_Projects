import threading
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory


class GesturePlayer:

    def __init__(self, node):
        self.node = node

        # Isolated node — completely separate from GestureManager
        self.action_node = rclpy.create_node('gesture_player_client')

        # Action client on isolated node
        self.client = ActionClient(
            self.action_node,
            FollowJointTrajectory,
            "/arm_controller/follow_joint_trajectory",
        )

        # Isolated executor spinning in its own thread permanently
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
                future.result()
                result_container[0] = True
                self.node.get_logger().info("Trajectory execution completed")
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
            # THIS RUNS IN ISOLATED EXECUTOR THREAD via timer
            # send_goal_async called here → future registered with isolated executor
            goal = FollowJointTrajectory.Goal()
            goal.trajectory = trajectory
            goal_future = self.client.send_goal_async(goal)
            goal_future.add_done_callback(on_goal_response)
            # Destroy this timer — only fire once
            if timer_container[0] is not None:
                self.action_node.destroy_timer(timer_container[0])

        # Add timer to isolated node — fires in isolated executor thread
        timer_container[0] = self.action_node.create_timer(0.01, submit_goal)

        # Wait for result — isolated executor handles everything
        done_event.wait(timeout=60.0)

        if result_container[0] is None:
            self.node.get_logger().error("Timeout: no result received")
            return False

        return result_container[0]
