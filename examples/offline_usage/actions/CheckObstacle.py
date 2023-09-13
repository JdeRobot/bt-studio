import py_trees
import sensor_msgs
from ....translator import tools

class CheckObstacle(py_trees.behaviour.Behaviour):

    def __init__(self, name: str = "CheckObstacle", ports = None):

        """Configure the name of the behaviour."""
        super().__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        self.ports = ports

    def setup(self, **kwargs: int) -> None:

        # Get the node passed from the tree
        try:
            self.node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(self.qualified_name)
            raise KeyError(error_message) from e  # 'direct cause' traceability

        self.subscription = self.node.create_subscription(
            sensor_msgs.msg.LaserScan,
            '/scan',
            self.listener_callback,
            10)
    
        self.scan = sensor_msgs.msg.LaserScan()
        self.counter = 0

    def initialise(self) -> None:

        """Reset a counter variable."""
        self.logger.debug("%s.initialise()" % (self.__class__.__name__))

    def listener_callback(self, msg):
        self.scan = msg

    def update(self) -> py_trees.common.Status:

        if len(self.scan.ranges) == 0: new_status = py_trees.common.Status.INVALID
        elif self.scan.ranges[0] > 1: 
            self.counter += 1
            try:
                gn_tools.set_port_content(self.ports["name1"], self.counter)
            except ValueError:
                print("Unable to set port content")
            new_status = py_trees.common.Status.SUCCESS
        else: new_status = py_trees.common.Status.FAILURE

        return new_status

    def terminate(self, new_status: py_trees.common.Status) -> None:

        """Nothing to clean up in this example."""

        self.logger.debug(

            "%s.terminate()[%s->%s]"

            % (self.__class__.__name__, self.status, new_status)

        )
