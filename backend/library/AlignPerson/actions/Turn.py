import py_trees
import geometry_msgs
import tree_tools


class Turn(py_trees.behaviour.Behaviour):
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

        # Setup the publisher for the robot speed
        self.publisher = self.node.create_publisher(
            msg_type=geometry_msgs.msg.Twist, topic="/cmd_vel", qos_profile=10
        )

    def initialise(self) -> None:
        """Executed when coming from an idle state"""

        # Debugging
        self.logger.debug("%s.initialise()" % (self.__class__.__name__))

    def update(self) -> py_trees.common.Status:
        """Executed when the action is ticked. Do not block!"""

        # Get values from the ports
        speed = tree_tools.get_port_content(self.ports["speed"])
        direction = tree_tools.get_port_content(self.ports["direction"])
        print(speed)
        print(direction)

        # Check if speed and direction are not empty before converting
        if speed and direction:
            # Publish the speed msg
            msg = geometry_msgs.msg.Twist()
            msg.angular.z = float(speed) * float(direction)
            self.publisher.publish(msg)
        else:
            self.logger.warning("Speed or direction is empty. Cannot publish message.")

        return py_trees.common.Status.RUNNING

    def terminate(self, new_status: py_trees.common.Status) -> None:
        """Called whenever the behaviour switches to a non-running state"""

        # Stop the robot
        msg = geometry_msgs.msg.Twist()
        msg.linear.x = 0.0
        self.publisher.publish(msg)

        # Debugging
        self.logger.debug(
            "%s.terminate()[%s->%s]"
            % (self.__class__.__name__, self.status, new_status)
        )
