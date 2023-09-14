import py_trees
import geometry_msgs
import std_msgs
from tree_translator import tools

class Turn(py_trees.behaviour.Behaviour):

    def __init__(self, name: str = "Turn", ports = None):

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

        self.logger.debug("%s.setup()" % (self.__class__.__name__))

        self.publisher = self.node.create_publisher(
            msg_type=geometry_msgs.msg.Twist,
            topic="/cmd_vel",
            qos_profile=10
        )

        self.publisher2 = self.node.create_publisher(
            msg_type=std_msgs.msg.String,
            topic="/n_obs",
            qos_profile=10
        )

        self.counter = 0

    def initialise(self) -> None:

        """Reset a counter variable."""

        self.logger.debug("%s.initialise()" % (self.__class__.__name__))

    def update(self) -> py_trees.common.Status:

        msg = geometry_msgs.msg.Twist()
        msg.angular.z = 0.4
        self.publisher.publish(msg)
        
        nobs = tools.get_port_content(self.ports["name2"])
        str_pub = std_msgs.msg.String()
        str_pub.data = str(nobs)
        self.publisher2.publish(str_pub)

        return py_trees.common.Status.RUNNING 

    def terminate(self, new_status: py_trees.common.Status) -> None:

        """Nothing to clean up in this example."""

        self.logger.debug(

            "%s.terminate()[%s->%s]"

            % (self.__class__.__name__, self.status, new_status)

        )
