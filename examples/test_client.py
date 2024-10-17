import rclpy
from typing import List, Optional
from rclpy.action import ActionClient
from rclpy.node import Node
from sd_external_msgs.action import TaskExecution
from sd_external_msgs.srv import GripperControl, Pause, Reset, Resume
from sd_external_msgs.msg import BoundingBox2DArray
from vision_msgs.msg import BoundingBox2D
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup


class TaskExecutionClient(Node):

    def __init__(self):
        super().__init__("task_execution_client")
        self._goal_handle = None
        self._feedback = None
        self._create_interafaces()

    def _create_interafaces(self):
        callback_group = ReentrantCallbackGroup()
        self._bbox_pub = self.create_publisher(
            BoundingBox2DArray, "bottles/bboxes", 1
        )
        self._gripper_client = self.create_client(
            GripperControl, "gripper/control", callback_group=callback_group
        )
        self._pause_client = self.create_client(
            Pause, "task/pause", callback_group=callback_group
        )
        self._reset_client = self.create_client(
            Reset, "task/reset", callback_group=callback_group
        )
        self._resume_client = self.create_client(
            Resume, "task/resume", callback_group=callback_group
        )
        self._task_client = ActionClient(
            self, TaskExecution, "task/execute", callback_group=callback_group
        )

    # public_methods
    def send_bboxes(self, item_ids: List[List[int]], bboxes: List[List[int]]):
        """
        Publishes a list of bounding boxes to a ROS2 topic.

        Args:
            bboxes (List[List[int]]): A list of bounding boxes, each defined by a list of four integers [x, y, width, height].

        Publishes the bounding boxes as an array message to the _bbox_pub topic.
        """
        array_msg = BoundingBox2DArray()

        for item_id, bbox in zip(item_ids, bboxes):
            bbox_msg = BoundingBox2D()
            bbox_msg.center.position.x = bbox[0]
            bbox_msg.center.position.y = bbox[1]
            bbox_msg.size_x = bbox[2]
            bbox_msg.size_y = bbox[3]

            # add item id and bounding box
            array_msg.ids.append(item_id)
            array_msg.bboxes.append(bbox_msg)

        self._bbox_pub.publish(array_msg)

    def control_gripper(self, right: Optional[int]=None, left: Optional[int]=None):
        """
        Sends a request to control the gripper(s). The value should be between 0 and 100, where 0 fully opens the gripper and 100 fully closes it.

        Args:
            right (Optional[int]): Value to control the right gripper (0 to 100). If None, the right gripper is not controlled.
            left (Optional[int]): Value to control the left gripper (0 to 100). If None, the left gripper is not controlled.

        Returns:
            GripperControl.Response: The response from the gripper control service.
        """
        request = GripperControl.Request()
        if right:
            request.right.should_use = True
            request.right.value = int(right)
        if left:
            request.left.should_use = True
            request.left.value = int(left)

        self.get_logger().info("control_gripper requesting...")
        result = self._gripper_client.call(request)
        self.get_logger().info("control_gripper done.")
        return result

    def pause(self):
        """
        Sends a request to pause the robot's current task.

        Returns:
            result (Pause.Response): The response from the pause service.
        """
        request = Pause.Request()
        self.get_logger().info("pause requesting...")
        result = self._pause_client.call(request)
        self.get_logger().info("pause done.")
        return result

    def resume(self):
        """
        Sends a request to resume the robot's paused task.

        Returns:
            result (Resume.Response): The response from the resume service.
        """
        request = Resume.Request()
        self.get_logger().info("resume requesting...")
        result = self._resume_client.call(request)
        self.get_logger().info("resume done.")
        return result

    def reset(self):
        """
        Sends a request to reset the robot's state.

        Returns:
            result (Reset.Response): The response from the reset service.
        """
        request = Reset.Request()
        self.get_logger().info("reset requesting...")
        result = self._reset_client.call(request)
        self.get_logger().info("reset done.")
        return result

    def send_task(
            self,
            task_list: List[str],
            bottle_color: str = "",
            target_mass: int = 0,
            bottle_bbox: List[int] = [0, 0, 0, 0],
    ):
        """
        Sends a task execution request to the task client.

        Args:
            task_list (List[str]): A list of task names (e.g., ["pick", "open", "pour"]).
            bottle_color (str, optional): The color of the bottle, defaults to an empty string.
            target_mass (int, optional): The target mass in grams, defaults to 0.
            bottle_bbox (List[int], optional): The bounding box of the bottle [x, y, width, height], defaults to [0, 0, 0, 0].

        Sends a goal to the task execution server and waits for the goal to complete.
        """
        # create goal msg
        goal_msg = TaskExecution.Goal()
        goal_msg.tasks = task_list      # ["pick", "open", "turn", "pour"] or ["pick"]
        goal_msg.color = bottle_color
        goal_msg.mass = target_mass
        goal_msg.bbox.center.position.x = float(bottle_bbox[0])
        goal_msg.bbox.center.position.y = float(bottle_bbox[1])
        goal_msg.bbox.size_x = float(bottle_bbox[2])
        goal_msg.bbox.size_y = float(bottle_bbox[3])

        self.get_logger().info("Waiting for the server...")
        self._task_client.wait_for_server()

        self.get_logger().info("Sending goal request...")
        self._send_goal_future = self._task_client.send_goal_async(
            goal_msg,
            feedback_callback=self._feedback_callback
        )
        self._send_goal_future.add_done_callback(self._goal_response_callback)

    def cancel_goal(self):
        """
        Cancels the current task execution goal.

        If there is an active goal, it sends a cancellation request to the server.
        """
        self.get_logger().info("Cancelling goal...")
        # Check if the goal_handle is valid before canceling
        if self._goal_handle is not None:
            cancel_future = self._goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self._cancel_done_callback)

    # private methods
    def _goal_response_callback(self, future):
        self._goal_handle = future.result()
        if not self._goal_handle.accepted:
            self.get_logger().info("Goal rejected.")
            return

        self.get_logger().info("Goal accepted.")

        # Wait for the result
        self._get_result_future = self._goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self._get_result_callback)

    def _get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"Result: {result.result}")
        # flushing self._goal_handle for the future call
        self._goal_handle = None

    def _feedback_callback(self, msg):
        self._feedback = msg.feedback.feedback
        self.get_logger().info(f"Feedback received: {self._feedback}")

    def _cancel_done_callback(self, future):
        cancel_response = future.result()
        if cancel_response.goals_canceling:
            self.get_logger().info("Goal successfully cancelled.")
        else:
            self.get_logger().info("Goal failed to cancel.")


def main(args=None):
    import threading
    import time

    rclpy.init(args=args)
    task_node = TaskExecutionClient()
    executor = MultiThreadedExecutor()
    executor.add_node(task_node)
    spin_thread = threading.Thread(target=executor.spin, args=())
    spin_thread.start()

    try:
        llm_task_list = ["pick", "open"]  # task list from LLM server
        bottle_color = "red"  # bottle to pick
        target_mass = 100  # target mass in gram
        center_x, center_y = 200, 200  # bounding box center
        size_x, size_y = 100, 100  # bounding box size
        bottle_bbox = [center_x, center_y, size_x, size_y]

        # send task
        task_node.send_task(
            task_list=llm_task_list,
            bottle_color=bottle_color,  # optional
            target_mass=target_mass,    # optional
            bottle_bbox=bottle_bbox,    # optional
        )

        # keeping the node alive
        iter = 0
        while True:

            if iter == 3:     # dummy call at iter 3
                task_node.control_gripper()
            elif iter == 5:   # dummy call at iter 5
                task_node.pause()
            elif iter == 10:  # dummy call at iter 10
                task_node.resume()
            elif iter == 15:  # dummy call at iter 15
                task_node.reset()

            iter += 1
            time.sleep(0.1)

    except KeyboardInterrupt:
        pass
    except Exception as e:
        print("Exception: ", e)
    finally:
        executor.remove_node(task_node)
        task_node.destroy_node()
        spin_thread.join()


if __name__ == "__main__":
    main()
