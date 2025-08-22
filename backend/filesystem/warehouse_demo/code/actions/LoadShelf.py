import py_trees
import geometry_msgs
import tree_tools
import math

class LoadShelf(py_trees.behaviour.Behaviour):

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

    def initialise(self) -> None:

        """ Executed when coming from an idle state """

        # Debugging
        self.logger.debug("%s.initialise()" % (self.__class__.__name__))

    def update(self) -> py_trees.common.Status:

        """ Executed when the action is ticked. Do not block! """

        mapImg = tree_tools.get_port_content(self.ports["map"])

        self.map_height, self.map_width, _ = mapImg.shape

        self.room_h = float(tree_tools.get_port_content(self.ports["map_h"]))
        self.room_w = float(tree_tools.get_port_content(self.ports["map_w"]))

        self.w_scale = self.map_width / self.room_w
        self.h_scale = self.map_height / self.room_h

        obj_x = float(tree_tools.get_port_content(self.ports["goal_x"]))
        obj_y = float(tree_tools.get_port_content(self.ports["goal_y"]))
        obj_width = int(tree_tools.get_port_content(self.ports["goal_width"]))
        obj_length = int(tree_tools.get_port_content(self.ports["goal_length"]))

        fcolor = 255 / 256  # float value of 255
        goalx, goaly = self.world2Pixels(obj_x, obj_y)
        for w in range(goaly - obj_width, goaly + obj_width, 1):
            for h in range(goalx - obj_length, goalx + obj_length, 1):
                mapImg[h, w] = (fcolor, fcolor, fcolor)

        tree_tools.set_port_content(self.ports["rob_width"], 20)
        tree_tools.set_port_content(self.ports["rob_length"], 9)
        tree_tools.set_port_content(self.ports["obj_x"], 0)
        tree_tools.set_port_content(self.ports["obj_y"], 0)
        tree_tools.set_port_content(self.ports["obj_yaw"], -math.pi)
        tree_tools.set_port_content(self.ports["map"], mapImg)
        return py_trees.common.Status.SUCCESS

    def world2Pixels(self, x, y):
        """
        Translates from World Coordinates to Pixels.
        """
        pixelx = (self.room_h / 2 - x) * self.h_scale
        pixely = (self.room_w / 2 - y) * self.w_scale

        return int(pixelx), int(pixely)

    def terminate(self, new_status: py_trees.common.Status) -> None:

        """ Called whenever the behaviour switches to a non-running state """

        # Debugging
        self.logger.debug("%s.terminate()[%s->%s]" % (self.__class__.__name__, self.status, new_status))