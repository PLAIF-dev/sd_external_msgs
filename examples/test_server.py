import rclpy
import time
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from sd_external_msgs.action import TaskExecution
from sd_external_msgs.srv import GripperControl, Pause, Reset, Resume
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup


class TaskExecutionServer(Node):

    def __init__(self):
        super().__init__('task_execution_server')
        self._create_interaces()
        self.get_logger().info("task_execution_server started.")

    def _create_interaces(self):
        callback_group = ReentrantCallbackGroup()
        self._task_server = ActionServer(
            self,
            TaskExecution,
            'task/execute',
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=ReentrantCallbackGroup()
        )
        self.create_service(
            GripperControl,
            "gripper/control",
            self._gripper_callback,
            callback_group=MutuallyExclusiveCallbackGroup()
        )
        self.create_service(
            Pause,
            "task/pause",
            self._pause_callback,
            callback_group=callback_group
        )
        self.create_service(
            Reset,
            "task/reset",
            self._reset_callback,
            callback_group=callback_group
        )
        self.create_service(
            Resume,
            "task/resume",
            self._resume_callback,
            callback_group=callback_group
        )

    def _gripper_callback(self, req: GripperControl.Request, res: GripperControl.Response):
        """Dummy Server."""
        self.get_logger().info("gripper request received.")
        return res

    def _pause_callback(self, req: Pause.Request, res: Pause.Response):
        """Dummy Server."""
        self.get_logger().info("pause request received.")
        return res

    def _reset_callback(self, req: Reset.Request, res: Reset.Response):
        """Dummy Server."""
        self.get_logger().info("reset request received.")
        return res

    def _resume_callback(self, req: Resume.Request, res: Resume.Response):
        """Dummy Server."""
        self.get_logger().info("resume request received.")
        return res

    async def _execute_callback(self, goal_handle):
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
            time.sleep(3)

        # Complete the goal
        goal_handle.succeed()
        result = TaskExecution.Result()
        result.result = 1  # Success
        self.get_logger().info('Goal succeeded')
        return result

    def _goal_callback(self, goal_request):
        self.get_logger().info(f'Received goal request with tasks: {goal_request.tasks}')
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle):
        self.get_logger().info('Received request to cancel goal')
        return CancelResponse.ACCEPT

    def destroy(self):
        self._task_server.destroy()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    task_execution_server = TaskExecutionServer()
    executor = MultiThreadedExecutor()
    executor.add_node(task_execution_server)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass

    executor.shutdown()
    task_execution_server.destroy()


if __name__ == '__main__':
    main()
