import py_trees
import geometry_msgs
import tree_tools
import sensor_msgs
import cv2
import cv_bridge
import numpy as np

# Color filter
red_mask = ([17, 15, 70], [50, 56, 255])
center_offset = 20
center_margin = 10
black_pixel = np.array([0, 0, 0])


def filter_img(raw_image, c_mask):

    lower = np.array(c_mask[0], dtype="uint8")
    upper = np.array(c_mask[1], dtype="uint8")

    mask = cv2.inRange(raw_image, lower, upper)
    f_img = cv2.bitwise_and(raw_image, raw_image, mask=mask)

    return f_img


# Gets a reference position between (center+offset, center+offset+margin)
def get_line_ref(img, offset, margin):

    height = img.shape[0]
    width = img.shape[1]

    center_row = int(height / 2) + offset

    c_x = 0
    c_y = 0
    npixels = 0

    for x in range(width):
        for y in range(center_row, center_row + margin):
            # Get pixel val and compare it with values for black
            pixel_val = img[y][x]
            comparison = pixel_val == black_pixel

            if not comparison.all():
                c_x += x
                c_y += y
                npixels += 1

    if npixels > 0:
        c_x /= npixels
        c_y /= npixels

    return (int(c_x), int(c_y))


def imageMsg2Image(img, bridge):

    if len(img.data) == 0:
        raise Exception("No Image")

    cv_image = 0
    cv_image = bridge.imgmsg_to_cv2(img, "bgr8")
    return cv_image


class DetectLine(py_trees.behaviour.Behaviour):

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

        # Setup the subscription to the laser
        self.subscription = self.node.create_subscription(
            sensor_msgs.msg.Image, "/cam_f1_left/image_raw", self.listener_callback, 10
        )

        self.last_img_ = sensor_msgs.msg.Image()
        self.bridge_ = cv_bridge.CvBridge()

    def listener_callback(self, msg):
        self.last_img_ = msg

    def initialise(self) -> None:
        """Executed when coming from an idle state"""

        # Debugging
        self.logger.debug("%s.initialise()" % (self.__class__.__name__))

    def update(self) -> py_trees.common.Status:
        """Executed when the action is ticked. Do not block!"""

        try:
            raw_img = imageMsg2Image(self.last_img_, self.bridge_)
        except:
            return py_trees.common.Status.FAILURE
        f_img = filter_img(raw_img, ([17, 15, 70], [50, 56, 255]))
        ref = get_line_ref(f_img, center_offset, center_margin)

        tree_tools.set_port_content(self.ports["center_x"], ref[0])
        tree_tools.set_port_content(self.ports["center_y"], ref[1])
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status: py_trees.common.Status) -> None:
        """Called whenever the behaviour switches to a non-running state"""

        # Debugging
        self.logger.debug(
            "%s.terminate()[%s->%s]"
            % (self.__class__.__name__, self.status, new_status)
        )
