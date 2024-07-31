import py_trees
from tree_gardener import tree_tools
from geometry_msgs.msg import PoseStamped
from vision_msgs.msg import Detection3DArray


class WaitPerson(py_trees.behaviour.Behaviour):
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

        # Setup the subscription to the vision msg
        self.subscription = self.node.create_subscription(
            Detection3DArray, "/output_detection_3d", self.listener_callback, 10
        )

        self.last_detection_ = Detection3DArray()

    def listener_callback(self, msg) -> None:
        self.last_detection_ = msg

    def initialise(self) -> None:
        """Executed when coming from an idle state"""

        # Debugging
        self.logger.debug("%s.initialise()" % (self.__class__.__name__))

    def update(self) -> py_trees.common.Status:
        """Executed when the action is ticked. Do not block!"""

        # Check the laser measures
        if self.last_detection_ == None:
            return py_trees.common.Status.RUNNING

        for detection in self.last_detection_.detections:
            if detection.results[0].hypothesis.class_id == "person":
                if detection.bbox.center.position.z < 1.5:
                    return py_trees.common.Status.SUCCESS
                self.logger.info(
                    "Client detected, depth %f" % (detection.bbox.center.position.z)
                )

        return py_trees.common.Status.RUNNING

    def terminate(self, new_status: py_trees.common.Status) -> None:
        """Called whenever the behaviour switches to a non-running state"""

        # Debugging
        self.logger.debug(
            "%s.terminate()[%s->%s]"
            % (self.__class__.__name__, self.status, new_status)
        )
