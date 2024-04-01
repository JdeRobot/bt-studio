import py_trees
import sensor_msgs
from tree_gardener import tree_tools

def check_obstacle_in_laser(laser_measures, amplitude):
        
        relevant_measures = laser_measures[:amplitude] + laser_measures[-amplitude:]

        for measure in relevant_measures:

            if measure < 1:
                return True
            
        return False

class CheckObstacle(py_trees.behaviour.Behaviour):

    def __init__(self, name, ports = None):

        """ Constructor, executed when the class is instantiated """

        # Configure the name of the behaviour
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
            sensor_msgs.msg.LaserScan,
            '/scan',
            self.listener_callback,
            10)
    
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
        if len(self.last_scan_.ranges) == 0: new_status = py_trees.common.Status.INVALID

        # Get params from ports
        amplitude = int(tree_tools.get_port_content(self.ports["amplitude"]))
        obs_dist = float(tree_tools.get_port_content(self.ports["obs_dist"]))
        
        # Check if there is an obstacle
        obstacle = check_obstacle_in_laser(self.last_scan_.ranges, amplitude)
        if obstacle: new_status = py_trees.common.Status.SUCCESS
        else: new_status = py_trees.common.Status.FAILURE

        return new_status

    def terminate(self, new_status: py_trees.common.Status) -> None:

        """ Called whenever the behaviour switches to a non-running state """

        # Debugging
        self.logger.debug("%s.terminate()[%s->%s]" % (self.__class__.__name__, self.status, new_status))