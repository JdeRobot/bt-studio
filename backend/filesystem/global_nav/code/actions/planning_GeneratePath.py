import py_trees
import geometry_msgs
import tree_tools
import math
from ompl import base as ob
from ompl import geometric as og
import matplotlib.pyplot as plt


def rotationMatrix(l1, l2, x, y, yaw):
    newx = x + (l1 * math.cos(yaw) - l2 * math.sin(yaw))
    newy = y + (l1 * math.sin(yaw) + l2 * math.cos(yaw))

    return int(newx), int(newy)


class planning_GeneratePath(py_trees.behaviour.Behaviour):

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

    def pixels2World(self, x, y):
        """
        Translates from Pixels to World Coordinates.
        """
        worldx = x / self.h_scale - self.room_h / 2
        worldy = y / self.w_scale - self.room_w / 2

        return worldx, worldy

    def world2Pixels(self, x, y):
        """
        Translates from World Coordinates to Pixels.
        """
        pixelx = (self.room_h / 2 - x) * self.h_scale
        pixely = (self.room_w / 2 - y) * self.w_scale

        return int(pixelx), int(pixely)

    def initialise(self) -> None:
        """Executed when coming from an idle state"""

        # Debugging
        self.logger.debug("%s.initialise()" % (self.__class__.__name__))

    def update(self) -> py_trees.common.Status:
        """Executed when the action is ticked. Do not block!"""
        # Publish the speed msg

        self.mapImg = tree_tools.get_port_content(self.ports["map"])

        self.map_height, self.map_width = self.mapImg.shape

        self.room_h = float(tree_tools.get_port_content(self.ports["map_h"]))
        self.room_w = float(tree_tools.get_port_content(self.ports["map_w"]))

        self.w_scale = self.map_width / self.room_w
        self.h_scale = self.map_height / self.room_h

        target_x = float(tree_tools.get_port_content(self.ports["target_x"]))
        target_y = float(tree_tools.get_port_content(self.ports["target_y"]))
        target_yaw = float(tree_tools.get_port_content(self.ports["target_yaw"]))

        x = float(tree_tools.get_port_content(self.ports["x"]))
        y = float(tree_tools.get_port_content(self.ports["y"]))
        yaw = float(tree_tools.get_port_content(self.ports["yaw"]))

        self.robotw = int(tree_tools.get_port_content(self.ports["robot_width"]))
        self.robotl = int(tree_tools.get_port_content(self.ports["robot_length"]))

        robotx, roboty = self.world2Pixels(x, y)
        goalx, goaly = self.world2Pixels(target_x, target_y)
        print(robotx, roboty, goalx, goaly)

        path = self.generatePath([[roboty, robotx], yaw], [[goalx, goaly], target_yaw])

        print(path)
        for index in range(len(path)):
            temp_x, temp_y = self.pixels2World(path[index][1], path[index][0])
            path[index] = [temp_x, temp_y]

        print(path)
        if len(path) == 0:
            return py_trees.common.Status.FAILURE
        else:
            tree_tools.set_port_content(self.ports["path"], path)
            return py_trees.common.Status.SUCCESS

    def terminate(self, new_status: py_trees.common.Status) -> None:
        """Called whenever the behaviour switches to a non-running state"""

        # Debugging
        self.logger.debug(
            "%s.terminate()[%s->%s]"
            % (self.__class__.__name__, self.status, new_status)
        )

    def generatePath(self, initial, end):
        # Instace the state space and add dimensions
        space = ob.DubinsStateSpace(1)
        bounds = ob.RealVectorBounds(2)
        bounds.setLow(0, 0)
        bounds.setLow(1, 0)
        bounds.setHigh(0, self.map_width)
        bounds.setHigh(1, self.map_height)
        space.setBounds(bounds)

        # Instace space information
        si = ob.SpaceInformation(space)
        si.setStateValidityChecker(ob.StateValidityCheckerFn(self.isStateValid))
        si.setStateValidityCheckingResolution(0.03)
        si.setup()

        # Create a problem instance
        pdef = ob.ProblemDefinition(si)

        # Set robot's starting and goal state
        start = ob.State(space)
        start[0] = initial[0][0]
        start[1] = initial[0][1]
        start[2] = initial[1]
        goal = ob.State(space)
        goal[0] = end[0][0]
        goal[1] = end[0][1]
        goal[2] = end[1]
        pdef.setStartAndGoalStates(start, goal, 0.01)

        # Set optimization objective
        pdef.setOptimizationObjective(ob.PathLengthOptimizationObjective(si))

        # Set optimal planner
        optimizingPlanner = og.LazyPRMstar(si)

        # Set the problem instance
        optimizingPlanner.setProblemDefinition(pdef)
        optimizingPlanner.setup()

        # Attempt to solve the planning problem in the given runtime
        solved = optimizingPlanner.solve(25.0)

        # If a solution is found path list will be fill
        path_lst = []
        if solved:
            p = pdef.getSolutionPath()
            for i in range(p.getStateCount()):
                path_lst.append(
                    [p.getState(i).getX(), p.getState(i).getY(), p.getState(i).getYaw()]
                )
        else:
            print("No solution found")
        return path_lst

    def isStateValid(self, state):
        w = min(int(state.getX()), self.map_width - 1)
        h = min(int(state.getY()), self.map_height - 1)
        yaw = state.getYaw()

        for x in range(-self.robotw, self.robotw, 1):
            for y in range(-self.robotl, self.robotl, 1):
                newx, newy = rotationMatrix(x, y, w, h, yaw)
                if 0 < newx < self.map_width and 0 < newy < self.map_height:
                    c = self.mapImg[newy, newx]
                    free = c * 255 > 200
                else:
                    free = False
                if not free:
                    break
            if not free:
                break

        return free
