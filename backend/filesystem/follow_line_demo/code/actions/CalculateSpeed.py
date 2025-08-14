import py_trees
import geometry_msgs
import tree_tools
import time


# PID class
class PID:
    def __init__(self, kp, kd, ki, st):

        self.kp = kp
        self.kd = kd
        self.ki = ki
        self.last_error = 0
        self.last_output = 0
        self.last_time = 0
        self.st = st
        self.last_sample = time.time()
        self.iterm = 0

    def set_lims(self, outmin, outmax):
        self.outmax = outmax
        self.outmin = outmin

    def calc(self, error):

        out = self.last_output

        # Calculate the time which has passed
        diff_time = time.time() - self.last_sample

        if diff_time >= self.st:
            # Derivative part
            diff_error = error - self.last_error

            # Integrative part (never higher than max)
            self.iterm += error * self.ki
            if self.iterm > self.outmax:
                self.iterm = self.outmax
            elif self.iterm < self.outmin:
                self.iterm = self.outmin

            # Output (never higher than max)
            out = self.kp * error + self.kd * diff_error + self.iterm
            if out > self.outmax:
                out = self.outmax
            elif out < self.outmin:
                out = self.outmin

            # Store info needed for next time
            self.last_error = error
            self.last_output = out
            self.last_sample = time.time()

        return out


class ErrBuff:
    def __init__(self, size):
        self.size = size
        self.ac_error = []
        self.nelems = 0
        self.next = 0

    def add(self, error):

        # Check if buffer has to cycle
        if self.next == self.size:
            self.next = 0

        # Checks if already big enough
        if self.nelems < self.size:
            self.ac_error.append(error)
            self.nelems += 1
        else:
            self.ac_error[self.next] = error

        self.next += 1

    def get_mean(self):
        mean = 0
        # print(self.ac_error)
        if self.nelems > 0:
            mean = sum(self.ac_error) / self.nelems

        return mean


# Color filter
center_offset = 20
center_margin = 10

# PID variables
direct = 0

# Angular pid
sp1 = 320
kp_1 = 0.01
kd_1 = 0.026
ki_1 = 0.00011
outmax_1 = 3
outmin_1 = -3.5

# Linear pid
sp2 = center_offset + int(center_margin / 2)
kp_2 = 0.04
kd_2 = 0.08
ki_2 = 0
outmax_2 = 6
outmin_2 = -6

# Car variables
max_linear = 11
max_lin_dec = 6
error_thres = 25

# PIDS objects (angular and linear speed)
pid1 = PID(kp_1, kd_1, ki_1, 0.03)
pid1.set_lims(outmin_1, outmax_1)

# buffer object
buff = ErrBuff(15)


class CalculateSpeed(py_trees.behaviour.Behaviour):

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

    def initialise(self) -> None:
        """Executed when coming from an idle state"""

        # Debugging
        self.logger.debug("%s.initialise()" % (self.__class__.__name__))

    def update(self) -> py_trees.common.Status:
        """Executed when the action is ticked. Do not block!"""

        try:
            center_x = float(tree_tools.get_port_content(self.ports["center_x"]))
            center_y = float(tree_tools.get_port_content(self.ports["center_y"]))
        except:
            tree_tools.set_port_content(self.ports["lin_speed"], 0)
            tree_tools.set_port_content(self.ports["ang_speed"], 0)
            return py_trees.common.Status.SUCCESS

        if (center_x, center_y) == (0, 0):
            tree_tools.set_port_content(self.ports["lin_speed"], 0)
            tree_tools.set_port_content(self.ports["ang_speed"], 0)
            return py_trees.common.Status.SUCCESS

        # Error calculation
        error = sp1 - center_x
        buff.add(abs(error))
        mean_error = round(buff.get_mean(), 2)

        norm_mean = mean_error / error_thres
        if norm_mean > 1:
            norm_mean = 1

        lin_speed = max_linear - max_lin_dec * norm_mean
        ang_speed = pid1.calc(error)

        tree_tools.set_port_content(self.ports["lin_speed"], lin_speed)
        tree_tools.set_port_content(self.ports["ang_speed"], ang_speed)
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status: py_trees.common.Status) -> None:
        """Called whenever the behaviour switches to a non-running state"""

        # Debugging
        self.logger.debug(
            "%s.terminate()[%s->%s]"
            % (self.__class__.__name__, self.status, new_status)
        )
