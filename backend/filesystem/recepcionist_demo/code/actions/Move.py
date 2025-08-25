import py_trees
import geometry_msgs
import rclpy
import time

import tree_tools
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped


class MoveAction(Node):
    def __init__(self):
        super().__init__("move_action_client")
        self.finished = False
        self.error = False

    def send_goal(self, goal):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose = goal

        self.start = time.perf_counter()

        self._action_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        self.finished = False
        self.error = False
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected :(")
            self.error = True
            return

        self.get_logger().info("Goal accepted :)")

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        self.finished = True

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback.distance_remaining
        print(feedback)
        if (
            feedback < 0.3
            and feedback != 0.0
            and not self.finished
            and (time.perf_counter() - self.start) > 2.0
        ):
            self.finished = True
            self._action_client.destroy()


class Move(py_trees.behaviour.Behaviour):
    def __init__(self, name, ports=None):
        """Constructor, executed when the class is instantiated"""

        # Configure the name of the behaviour
        super().__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))

        # Get the ports
        self.ports = ports
        self.action = MoveAction()

    def setup(self, **kwargs) -> None:
        """Executed when the setup function is called upon the tree"""

        # Get the node passed from the tree (needed for interaction with ROS)
        try:
            self.node = kwargs["node"]
        except KeyError as e:
            error_message = "Couldn't find the tree node"
            raise KeyError(error_message) from e

    def initialise(self) -> None:
        """Executed when coming from an idle state"""

        # Debugging
        self.logger.debug("%s.initialise()" % (self.__class__.__name__))
        self.action = MoveAction()
        self.action.send_goal(tree_tools.get_port_content(self.ports["waypoint"]))

    def update(self) -> py_trees.common.Status:
        """Executed when the action is ticked. Do not block!"""
        rclpy.spin_once(self.action)

        if self.action.finished:
            return py_trees.common.Status.SUCCESS

        if self.action.error:
            return py_trees.common.Status.FAILURE

        return py_trees.common.Status.RUNNING

    def terminate(self, new_status: py_trees.common.Status) -> None:
        """Called whenever the behaviour switches to a non-running state"""

        # Debugging
        self.logger.debug(
            "%s.terminate()[%s-&gt;%s]"
            % (self.__class__.__name__, self.status, new_status)
        )
