import py_trees
import geometry_msgs
import tree_tools

class FinalVFF(py_trees.behaviour.Behaviour):

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

        goal_f = tree_tools.get_port_content(self.ports["goal"])
        obs_f = tree_tools.get_port_content(self.ports["obstacle"])

        ALPHA = 1
        BETA = 1.5

        f_x = ALPHA * goal_f[0] + BETA * obs_f[0]

        f_y = ALPHA * goal_f[1] + BETA * obs_f[1]

        tree_tools.set_port_content(self.ports["lin"], f_x)
        tree_tools.set_port_content(self.ports["ang"], f_y)
        return py_trees.common.Status.SUCCESS 

    def terminate(self, new_status: py_trees.common.Status) -> None:

        """ Called whenever the behaviour switches to a non-running state """

        # Debugging
        self.logger.debug("%s.terminate()[%s->%s]" % (self.__class__.__name__, self.status, new_status))