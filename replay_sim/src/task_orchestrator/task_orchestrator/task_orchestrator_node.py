import time
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from gesture_interface_msgs.srv import ExecuteTask, PlayGesture


class TaskOrchestrator(Node):

    def __init__(self):
        super().__init__('task_orchestrator')

        self.cb_group = ReentrantCallbackGroup()

        self.task_map = {
            'align_t': 'align_t_task',
        }

        self.srv = self.create_service(
            ExecuteTask,
            '/execute_task',
            self.execute_task_callback,
            callback_group=self.cb_group
        )

        self.play_client = self.create_client(
            PlayGesture,
            '/play_gesture',
            callback_group=self.cb_group
        )

        self.get_logger().info('Task Orchestrator started')
        self.get_logger().info(f'Known tasks: {list(self.task_map.keys())}')

    def execute_task_callback(self, request, response):

        task_name = request.task_name
        self.get_logger().info(f'Received task request: {task_name}')

        # Check task exists
        if task_name not in self.task_map:
            response.success = False
            response.message = (
                f"Unknown task '{task_name}'. "
                f"Available: {list(self.task_map.keys())}"
            )
            self.get_logger().error(response.message)
            return response

        gesture_name = self.task_map[task_name]
        self.get_logger().info(f'Mapped to gesture: {gesture_name}')

        # Wait for service
        if not self.play_client.wait_for_service(timeout_sec=5.0):
            response.success = False
            response.message = '/play_gesture service not available'
            self.get_logger().error(response.message)
            return response

        # Call play_gesture async
        play_request = PlayGesture.Request()
        play_request.name = gesture_name
        future = self.play_client.call_async(play_request)

        # Wait with timeout — do NOT call spin_once inside MultiThreadedExecutor
        timeout_sec = 60.0
        start_time = time.time()
        while not future.done():
            if (time.time() - start_time) > timeout_sec:
                response.success = False
                response.message = f'Timeout waiting for gesture: {gesture_name}'
                self.get_logger().error(response.message)
                return response
            time.sleep(0.1)

        play_response = future.result()

        if play_response is None:
            response.success = False
            response.message = f'Null response for gesture: {gesture_name}'
            self.get_logger().error(response.message)
            return response

        if play_response.success:
            response.success = True
            response.message = (
                f"Task '{task_name}' completed via gesture '{gesture_name}'"
            )
            self.get_logger().info(response.message)
        else:
            response.success = False
            response.message = f"Task failed: {play_response.message}"
            self.get_logger().error(response.message)

        return response


def main(args=None):
    rclpy.init(args=args)
    node = TaskOrchestrator()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
