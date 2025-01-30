import py_trees
import tree_tools
from geometry_msgs.msg import PoseStamped


class SetDestination(py_trees.behaviour.Behaviour):
    def __init__(self, name, ports=None):
        """Constructor, executed when the class is instantiated"""

        # Configure the name of the behaviour
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
        wp_id = tree_tools.get_port_content(self.ports["waypoint_id"])

        goal = PoseStamped()
        goal.header.frame_id = "map"

        if wp_id == "Door":
            print("Go to Door")
            goal.pose.position.x = 5.8
            goal.pose.position.y = -4.25

            goal.pose.orientation.x = 0.0
            goal.pose.orientation.y = 0.0
            goal.pose.orientation.z = 0.0
            goal.pose.orientation.w = 0.0
        elif wp_id == "Couch":
            print("Go to Couch")
            goal.pose.position.x = 1.2
            goal.pose.position.y = -0.5

            goal.pose.orientation.x = 0.0
            goal.pose.orientation.y = 0.0
            goal.pose.orientation.z = 0.7
            goal.pose.orientation.w = -0.7
        elif wp_id == "Kitchen":
            print("Go to Kitchen")
            goal.pose.position.x = 7.75
            goal.pose.position.y = -3.0

            goal.pose.orientation.x = 0.0
            goal.pose.orientation.y = 0.0
            goal.pose.orientation.z = 0.0
            goal.pose.orientation.w = 0.0
        elif wp_id == "Gym":
            print("Passing through the gym")
            goal.pose.position.x = 1.2
            goal.pose.position.y = 2.0

            goal.pose.orientation.x = 0.0
            goal.pose.orientation.y = 0.0
            goal.pose.orientation.z = 0.0
            goal.pose.orientation.w = 0.0
        else:
            goal.pose.position.x = 0.0
            goal.pose.position.y = 0.0

            goal.pose.orientation.x = 0.0
            goal.pose.orientation.y = 0.0
            goal.pose.orientation.z = 0.0
            goal.pose.orientation.w = 0.0

        tree_tools.set_port_content(self.ports["waypoint"], goal)
        print(goal)

        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status: py_trees.common.Status) -> None:
        """Called whenever the behaviour switches to a non-running state"""

        # Debugging
        self.logger.debug(
            "%s.terminate()[%s->%s]"
            % (self.__class__.__name__, self.status, new_status)
        )
