import py_trees
import geometry_msgs
import tree_tools
import math
import nav_msgs.msg

# Enter sequential code!
MAX_SPEED = 3
MAX_RANGE = 3
MIN_RANGE = 0.8


def get_module(vector):

    x = vector[0]
    y = vector[1]

    module = math.sqrt(x**2 + y**2)

    return module


def absolute2relative(x_abs, y_abs, robotx, roboty, robott):

    # robotx, roboty are the absolute coordinates of the robot
    # robott is its absolute orientation
    # Convert to relatives
    dx = x_abs - robotx
    dy = y_abs - roboty

    # Rotate with current angle
    x_rel = dx * math.cos(-robott) - dy * math.sin(-robott)
    y_rel = dx * math.sin(-robott) + dy * math.cos(-robott)

    return [x_rel, y_rel]


def get_target_pos(target, r_x, r_y, r_t):

    # gets the target coords respect to the robot

    t_abs_x = target[0]
    t_abs_y = target[1]

    [target_x, target_y] = absolute2relative(t_abs_x, t_abs_y, r_x, r_y, r_t)

    return [target_x, target_y]


def get_car_force(target, r_x, r_y, r_t):

    [target_x, target_y] = get_target_pos(target, r_x, r_y, r_t)

    target_module = get_module([target_x, target_y])

    # get the angle of the target respect to the robot
    if target_x != 0:
        car_angle = math.atan(target_y / target_x)
    else:
        car_angle = math.pi / 2
        if target_y < 0:
            car_angle *= -1

    # contain the module of the target within the maximum speed
    if target_module > MAX_RANGE:
        target_module = MAX_SPEED

    # gets the coords of the target, with the module contained
    car_x = target_module * math.cos(car_angle)
    car_y = target_module * math.sin(car_angle)

    # when the target is behind the robot, the coords will be backwards
    if target_x < 0:
        car_x *= -1
        car_y *= -1

    return [car_x, car_y]


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
        rotateZ = math.atan2(rotateZa0, rotateZa1)

    return rotateZ


class GoalVFF(py_trees.behaviour.Behaviour):

    def __init__(self, name, ports=None):
        """Constructor, executed when the class is instantiated"""

        # Configure the name of the behavioure
        super().__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))

        # Get the ports
        self.ports = ports
        self.target_index = 0
        self.targets = [
            (0, -30),
            (-6, -41),
            (-14, -31),
            (-16, -13),
            (-55, -12),
            (-63, -30),
            (-72, -35),
            (-106, -8),
            (-68, 38),
            (-47, 45),
            (-38, 64),
            (-26, 43),
            (-17, 58),
            (-3, 58),
            (0, 13),
        ]
        self.current_target = self.targets[self.target_index]

    def setup(self, **kwargs: int) -> None:
        """Executed when the setup function is called upon the tree"""

        # Get the node passed from the tree (needed for interaction with ROS)
        try:
            self.node = kwargs["node"]
        except KeyError as e:
            error_message = "Couldn't find the tree node"
            raise KeyError(error_message) from e

        self.subscription = self.node.create_subscription(
            nav_msgs.msg.Odometry, "/odom", self.listener_callback, 10
        )

        self.last_scan_ = nav_msgs.msg.Odometry()

    def listener_callback(self, msg) -> None:
        self.last_scan_ = msg

    def initialise(self) -> None:
        """Executed when coming from an idle state"""

        # Debugging
        self.logger.debug("%s.initialise()" % (self.__class__.__name__))

    def update(self) -> py_trees.common.Status:
        """Executed when the action is ticked. Do not block!"""

        x = self.last_scan_.pose.pose.position.x
        y = self.last_scan_.pose.pose.position.y

        ori = self.last_scan_.pose.pose.orientation
        yaw = quat2Yaw(ori.w, ori.x, ori.y, ori.z)

        force = get_target_pos(self.current_target, x, y, yaw)
        t_module = get_module(force)

        if t_module < MIN_RANGE:
            self.target_index = self.target_index + 1 % len(self.targets)
            self.current_target = self.targets[self.target_index]

        tree_tools.set_port_content(
            self.ports["vff"], get_car_force(self.current_target, x, y, yaw)
        )
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status: py_trees.common.Status) -> None:
        """Called whenever the behaviour switches to a non-running state"""

        # Debugging
        self.logger.debug(
            "%s.terminate()[%s->%s]"
            % (self.__class__.__name__, self.status, new_status)
        )
