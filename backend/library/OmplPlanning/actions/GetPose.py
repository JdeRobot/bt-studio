import py_trees
import geometry_msgs
import tree_tools
import nav_msgs.msg
from math import atan2

def quat2Yaw(qw, qx, qy, qz):
    """
    Translates from Quaternion to Yaw.
    @param qw,qx,qy,qz: Quaternion values
    @type qw,qx,qy,qz: float
    @return Yaw value translated from Quaternion
    """

    rotateZa0 = 2.0 * (qx * qy + qw * qz)
    rotateZa1 = qw * qw + qx * qx - qy * qy - qz * qz
    rotateZ = 0.0
    if rotateZa0 != 0.0 and rotateZa1 != 0.0:
        rotateZ = atan2(rotateZa0, rotateZa1)

    return rotateZ

class GetPose(py_trees.behaviour.Behaviour):

    def __init__(self, name, ports = None):

        """ Constructor, executed when the class is instantiated """

        # Configure the name of the behavioure
        super().__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))

        # Get the ports
        self.ports = ports

    def setup(self, **kwargs: int) -> None:

        """ Executed when the setup function is called upon the tree """

        # Get the node passed from the tree (needed for interaction with ROS)
        try:
            self.node = kwargs['node']
        except KeyError as e:
            error_message = "Couldn't find the tree node"
            raise KeyError(error_message) from e

        topic = tree_tools.get_port_content(self.ports["topic"])
        self.subscription = self.node.create_subscription(
            nav_msgs.msg.Odometry, topic, self.listener_callback, 10
        )

        self.last_scan_ = nav_msgs.msg.Odometry()

    def listener_callback(self, msg) -> None:
        self.last_scan_ = msg

    def initialise(self) -> None:

        """ Executed when coming from an idle state """

        # Debugging
        self.logger.debug("%s.initialise()" % (self.__class__.__name__))

    def update(self) -> py_trees.common.Status:

        """ Executed when the action is ticked. Do not block! """

        x = self.last_scan_.pose.pose.position.x
        y = self.last_scan_.pose.pose.position.y

        ori = self.last_scan_.pose.pose.orientation
        yaw = quat2Yaw(ori.w, ori.x, ori.y, ori.z)

        tree_tools.set_port_content(self.ports["x"], x)
        tree_tools.set_port_content(self.ports["y"], y)
        tree_tools.set_port_content(self.ports["yaw"], yaw)
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status: py_trees.common.Status) -> None:

        """ Called whenever the behaviour switches to a non-running state """

        # Debugging
        self.logger.debug("%s.terminate()[%s->%s]" % (self.__class__.__name__, self.status, new_status))