import py_trees
import sensor_msgs
import tree_tools
import std_msgs
from cv_bridge import CvBridge
import sensor_msgs
import cv2
import numpy as np


class DetectPerson(py_trees.behaviour.Behaviour):
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

        # Setup the subscription to camera
        self.subscription = self.node.create_subscription(
            sensor_msgs.msg.Image, "/depth_camera/image_raw", self.listener_callback, 10
        )

        self.bridge = CvBridge()
        self.img_received = False

    def listener_callback(self, msg):
        self.last_img = self.bridge.imgmsg_to_cv2(msg)
        self.img_received = True

    def initialise(self) -> None:
        """Executed when coming from an idle state"""

        # Debugging
        self.logger.debug("%s.initialise()" % (self.__class__.__name__))

    def update(self) -> py_trees.common.Status:
        """Executed when the action is ticked. Do not block!"""
        new_status = py_trees.common.Status.FAILURE

        if self.img_received:
            # It converts the BGR color space of image to HSV color space
            img = self.last_img
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

            # Threshold of blue in HSV space
            lower_blue = np.array([40, 40, 20])
            upper_blue = np.array([70, 255, 255])

            # preparing the mask to overlay
            mask = cv2.inRange(hsv, lower_blue, upper_blue)

            # Apply the mask
            result = cv2.bitwise_and(img, img, mask=mask)

            # Calculate green blob center
            gray_image = cv2.cvtColor(result, cv2.COLOR_BGR2GRAY)
            ret, thresh = cv2.threshold(gray_image, 127, 255, 0)
            M = cv2.moments(thresh)

            cX = -1
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                if cX < img.shape[1] / 2:
                    tree_tools.set_port_content(self.ports["direction"], 1)
                else:
                    tree_tools.set_port_content(self.ports["direction"], -1)

                new_status = py_trees.common.Status.SUCCESS

        return new_status

    def terminate(self, new_status: py_trees.common.Status) -> None:
        """Called whenever the behaviour switches to a non-running state"""

        # Debugging
        self.logger.debug(
            "%s.terminate()[%s->%s]"
            % (self.__class__.__name__, self.status, new_status)
        )
