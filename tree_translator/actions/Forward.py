import py_trees
import geometry_msgs
import std_msgs
from tree_gardener import tree_tools
from cv_bridge import CvBridge
import sensor_msgs
import cv2

class Forward(py_trees.behaviour.Behaviour):

    def __init__(self, name, ports = None):

        """ Constructor, executed when the class is instantiated """

        # Configure the name of the behaviour
        super().__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))

        # Get the ports
        self.ports = ports

    def setup(self, **kwargs) -> None:

        """ Executed when the setup function is called upon the tree """

        # Get the node passed from the tree (needed for interaction with ROS)
        try:
            self.node = kwargs['node']
        except KeyError as e:
            error_message = "Couldn't find the tree node"
            raise KeyError(error_message) from e

        # Setup the publisher for the robot speed
        self.publisher = self.node.create_publisher(
            msg_type=geometry_msgs.msg.Twist,
            topic="/cmd_vel",
            qos_profile=10
        )

        # Setup the publisher for n_obs
        self.publisher2 = self.node.create_publisher(
            msg_type=std_msgs.msg.String,
            topic="/n_obs",
            qos_profile=10
        )

        # Setup the subscription to camera
        self.subscription = self.node.create_subscription(
            sensor_msgs.msg.Image,
            '/camera/image_raw',
            self.listener_callback,
            10
        )

        self.bridge = CvBridge()
        self.img_received = False
    
    def listener_callback(self, msg):

        self.last_img = self.bridge.imgmsg_to_cv2(msg)
        self.img_received = True

    def initialise(self) -> None:

        """ Executed when coming from an idle state """

        # Debugging
        self.logger.debug("%s.initialise()" % (self.__class__.__name__))

    def update(self) -> py_trees.common.Status:

        """ Executed when the action is ticked. Do not block! """

        # Publish the speed msg
        msg = geometry_msgs.msg.Twist()
        msg.linear.x = float(tree_tools.get_port_content(self.ports["speed"]))
        self.publisher.publish(msg)

        # Publish the number of obstacles retrieved from the port
        nobs = tree_tools.get_port_content(self.ports["obs_port"])
        str_pub = std_msgs.msg.String()
        str_pub.data = str(nobs)
        self.publisher2.publish(str_pub)

        if self.img_received:
            cv2.imshow("Robot img", self.last_img)
            cv2.waitKey(1)

        return py_trees.common.Status.RUNNING 

    def terminate(self, new_status: py_trees.common.Status) -> None:

        """ Called whenever the behaviour switches to a non-running state """

        # Stop the robot
        msg = geometry_msgs.msg.Twist()
        msg.linear.x = 0.0
        self.publisher.publish(msg)

        # Debugging
        self.logger.debug("%s.terminate()[%s->%s]" % (self.__class__.__name__, self.status, new_status))