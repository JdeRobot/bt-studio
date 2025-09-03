import py_trees
import sensor_msgs
import tree_tools
import numpy as np
import math

class ObstacleVFF(py_trees.behaviour.Behaviour):

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

        # Setup the subscription to the laser
        self.subscription = self.node.create_subscription(
            sensor_msgs.msg.LaserScan, "/f1/laser/scan", self.listener_callback, 10
        )

        self.last_scan_ = sensor_msgs.msg.LaserScan()

    def listener_callback(self, msg) -> None:
        self.last_scan_ = msg

    def initialise(self) -> None:

        """ Executed when coming from an idle state """

        # Debugging
        self.logger.debug("%s.initialise()" % (self.__class__.__name__))

    def update(self) -> py_trees.common.Status:

        """ Executed when the action is ticked. Do not block! """

        # Check the laser measures
        if len(self.last_scan_.ranges) == 0:
            return py_trees.common.Status.INVALID

        laser_data = self.last_scan_
        laser = []
        i = 0

        while i < 180:
            dist = laser_data.ranges[i]
            pos = i - 90  # To get the angle 0 in the middle-front of the robot

            angle = math.radians(pos)

            # It is a e^x type of function
            importance = 15 / (2.7 ** (2 * dist))  # define the function that relates
            # the distance to the object to the importance that reading gets,
            # the nearest the obstacle, the greater the importance.

            # get the components of the readings
            x = importance * math.cos(angle) * -1
            y = importance * math.sin(angle) * -1
            v = (x, y)

            laser += [v]
            i += 1

        # the mean will show the repulsion force because of the obstacles
        laser_mean = np.mean(laser, axis=0)

        obs_x = laser_mean[0]
        obs_y = laser_mean[1]

        tree_tools.set_port_content(self.ports["vff"], (obs_x, obs_y))
        return py_trees.common.Status.SUCCESS 

    def terminate(self, new_status: py_trees.common.Status) -> None:

        """ Called whenever the behaviour switches to a non-running state """

        # Debugging
        self.logger.debug("%s.terminate()[%s->%s]" % (self.__class__.__name__, self.status, new_status))