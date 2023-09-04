import py_trees
import geometry_msgs
from py_gardener import gn_tools

class Forward(py_trees.behaviour.Behaviour):

    def __init__(self, name: str = "Move", port = None):

        """Configure the name of the behaviour."""
        super().__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        self.port = port

    def setup(self, **kwargs) -> None:

        # Get the node passed from the tree
        try:
            self.node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(self.qualified_name)
            raise KeyError(error_message) from e  # 'direct cause' traceability

        self.publisher = self.node.create_publisher(
            msg_type=geometry_msgs.msg.Twist,
            topic="/cmd_vel",
            qos_profile=10
        )

    def initialise(self) -> None:

        """Reset a counter variable."""
        self.logger.debug("%s.initialise()" % (self.__class__.__name__))

    def update(self) -> py_trees.common.Status:

        msg = geometry_msgs.msg.Twist()
        msg.linear.x = float(gn_tools.get_port_content(self.port["speed"]))
        self.publisher.publish(msg)

        return py_trees.common.Status.RUNNING 

    def terminate(self, new_status: py_trees.common.Status) -> None:

        """Nothing to clean up in this example."""
        msg = geometry_msgs.msg.Twist()
        msg.linear.x = 0.0
        self.publisher.publish(msg)

        self.logger.debug(

            "%s.terminate()[%s->%s]"

            % (self.__class__.__name__, self.status, new_status)

        )