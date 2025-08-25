import py_trees
import geometry_msgs
import tree_tools
import math

from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy import spin_until_future_complete, ok
import threading
from gazebo_msgs.srv import ApplyJointEffort
from std_msgs.msg import String
import time

DEFAULT = 2


def cmdLift2String(cmdLift):
    return cmdLift.msg


class CMDLift:
    def __init__(self):
        self.msg = String()
        self.msg.data = "unload"

    def cmd(self, cmd):
        self.msg.data = cmd

    def __str__(self):
        return "CMDlift:" + self.msg.data


class PlatformController(Node):
    def __init__(self, joint):
        """
        PlatformController Constructor.
        @param joint: Joint name to move
        @type joint: String
        """
        super().__init__("platform_controller")
        self.lock = threading.Lock()

        # Joint Effort Service
        self.client = self.create_client(ApplyJointEffort, "/apply_joint_effort")
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")

        self.req = ApplyJointEffort.Request()
        self.req.joint_name = joint

        self.future = None
        self.future_lock = threading.Lock()

    def send_effort(self, input_effort):

        self.req.effort = input_effort
        self.req.start_time.sec = 0
        self.req.start_time.nanosec = 0
        self.req.duration.sec = 2000
        self.req.duration.nanosec = 0

        future = self.client.call_async(self.req)
        spin_until_future_complete(self, future)

        self.future_lock.acquire()
        self.future = future
        # print("self.future: ",self.future.result().success, self.future.result().status_message)
        self.future_lock.release()


class PlatformCommandNode(Node):
    def __init__(self, topic):
        super().__init__("platform_command_listener")
        self.sub = self.create_subscription(String, topic, self.listener_callback, 10)
        self.applied_effort = 0
        self.controller = PlatformController("lift_joint")

    def listener_callback(self, event):
        """
        Load/Unload logic so that effort is not accumulated in joint
        """
        command = event.data
        if command == "load":
            if self.applied_effort == 0:
                effort = DEFAULT
            elif self.applied_effort < 0:
                effort = self.applied_effort * -2
            else:
                return
        elif command == "unload":
            if self.applied_effort == DEFAULT:
                effort = -DEFAULT
            elif self.applied_effort > 0:
                effort = self.applied_effort * -2
            else:
                return
        else:
            return

        self.controller.send_effort(float(effort))
        self.applied_effort = self.applied_effort + effort


platform_listener = PlatformCommandNode("/send_effort")
executor = MultiThreadedExecutor()
executor.add_node(platform_listener)


def __auto_spin() -> None:
    while ok():
        try:
            executor.spin_once(timeout_sec=0)
        except:
            pass
        time.sleep(1 / 30)


executor_thread = threading.Thread(target=__auto_spin, daemon=True)
executor_thread.start()


class LoadShelf(py_trees.behaviour.Behaviour):

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
        self.data = CMDLift()
        self.publisher = self.node.create_publisher(String, "/send_effort", 10)

    def initialise(self) -> None:
        """Executed when coming from an idle state"""

        # Debugging
        self.logger.debug("%s.initialise()" % (self.__class__.__name__))

    def update(self) -> py_trees.common.Status:
        """Executed when the action is ticked. Do not block!"""

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

        self.data.cmd("load")
        self.publisher.publish(cmdLift2String(self.data))

        return py_trees.common.Status.SUCCESS

    def world2Pixels(self, x, y):
        """
        Translates from World Coordinates to Pixels.
        """
        pixelx = (self.room_h / 2 - x) * self.h_scale
        pixely = (self.room_w / 2 - y) * self.w_scale

        return int(pixelx), int(pixely)

    def terminate(self, new_status: py_trees.common.Status) -> None:
        """Called whenever the behaviour switches to a non-running state"""

        # Debugging
        self.logger.debug(
            "%s.terminate()[%s->%s]"
            % (self.__class__.__name__, self.status, new_status)
        )
