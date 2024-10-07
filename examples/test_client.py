import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from sd_external_msgs.action import TaskExecution


class TaskExecutionClient(Node):

    def __init__(self):
        super().__init__('task_execution_client')
        self._action_client = ActionClient(self, TaskExecution, 'task_execution')

    def send_goal(self):
        goal_msg = TaskExecution.Goal()

        # Define a list of tasks to execute
        goal_msg.tasks = ['Task1', 'Task2', 'Task3']

        # Wait until the action server is available
        self._action_client.wait_for_server()

        self.get_logger().info('Sending goal request...')

        # Send the goal request
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected.')
            return

        self.get_logger().info('Goal accepted.')

        # Wait for the result
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result.result}')

        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Feedback received: {feedback}')


def main(args=None):
    rclpy.init(args=args)

    task_execution_client = TaskExecutionClient()
    task_execution_client.send_goal()

    rclpy.spin(task_execution_client)


if __name__ == '__main__':
    main()
