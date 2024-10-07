import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from sd_external_msgs.action import TaskExecution


class TaskExecutionServer(Node):

    def __init__(self):
        super().__init__('task_execution_server')
        self.get_logger().info("task_execution_server started.")

        self._action_server = ActionServer(
            self,
            TaskExecution,
            'task_execution',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

    def goal_callback(self, goal_request):
        """Callback to accept or reject a client request to start an action."""
        self.get_logger().info(f'Received goal request with tasks: {goal_request.tasks}')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Callback to accept or reject a client request to cancel an action."""
        self.get_logger().info('Received request to cancel goal')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        """Execute the goal by processing the tasks and providing feedback."""
        self.get_logger().info('Executing goal...')

        tasks = goal_handle.request.tasks
        feedback_msg = TaskExecution.Feedback()

        # Process tasks
        for i, task in enumerate(tasks):
            if goal_handle.is_cancel_requested:
                self.get_logger().info('Goal canceled')
                goal_handle.canceled()
                result = TaskExecution.Result()
                result.result = 2  # Canceled
                return result

            self.get_logger().info(f'Processing task {i+1}: {task}')
            feedback_msg.feedback = 1
            goal_handle.publish_feedback(feedback_msg)

            # Simulate some work
            time.sleep(2)

        # Complete the goal
        goal_handle.succeed()
        result = TaskExecution.Result()
        result.result = 1  # Success
        self.get_logger().info('Goal succeeded')
        return result

    def destroy(self):
        self._action_server.destroy()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    task_execution_server = TaskExecutionServer()

    try:
        rclpy.spin(task_execution_server)
    except KeyboardInterrupt:
        pass

    task_execution_server.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
