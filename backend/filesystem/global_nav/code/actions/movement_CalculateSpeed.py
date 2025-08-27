import py_trees
import geometry_msgs
import tree_tools
from math import atan2, pi


def getSign(value):
    if value < 0:
        return -1
    return 1


def getTurnDirection(robot_yaw, goal_yaw):
    if goal_yaw > robot_yaw:
        direction = 1
    else:
        direction = -1

    if getSign(goal_yaw) != getSign(robot_yaw):
        a = abs(goal_yaw) + abs(robot_yaw)
        b = 2 * pi - a
        if b < a:
            direction = -direction

    return direction


class movement_CalculateSpeed(py_trees.behaviour.Behaviour):

    def __init__(self, name, ports=None):
        """Constructor, executed when the class is instantiated"""

        # Configure the name of the behavioure
        super().__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))

        # Get the ports
        self.ports = ports

    def setup(self, **kwargs: int) -> None:
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

    def update(self) -> py_trees.common.Status:
        """Executed when the action is ticked. Do not block!"""

        # Publish the speed msg
        target_x = float(tree_tools.get_port_content(self.ports["target_x"]))
        target_y = float(tree_tools.get_port_content(self.ports["target_y"]))

        x = float(tree_tools.get_port_content(self.ports["x"]))
        y = float(tree_tools.get_port_content(self.ports["y"]))
        yaw = float(tree_tools.get_port_content(self.ports["yaw"]))

        x_diff = target_x - x
        y_diff = target_y - y
        target_yaw = atan2(y_diff, x_diff)

        if abs(target_yaw - yaw) > 0.3:  # No aligned
            ang_speed = getTurnDirection(yaw, target_yaw) * 0.9
        else:  # Aligned
            ang_speed = 0.0

        if ang_speed == 0:
            lin_speed = 3
        else:
            lin_speed = 0

        tree_tools.set_port_content(self.ports["lin"], lin_speed)
        tree_tools.set_port_content(self.ports["ang"], ang_speed)

        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status: py_trees.common.Status) -> None:
        """Called whenever the behaviour switches to a non-running state"""

        # Debugging
        self.logger.debug(
            "%s.terminate()[%s->%s]"
            % (self.__class__.__name__, self.status, new_status)
        )
