
import multiprocessing

from rclpy.node import Node
from geometry_msgs.msg import Point
from src.comm.mavlink_node import create_command


class LandingTargetSender(Node):
    """
    A ROS 2 node that publishes the landing target position to a topic.
    """
    def __init__(self, topic: str = "/erc/landing_target_filtered",
                 command_queue: multiprocessing.Queue, telemetry_queue: multiprocessing.Queue,
                 landing_target_id: int = 102):
        super().__init__("landing_target_sender")
        self.create_subscription(
            Point, f"{topic}", self.vision_callback, 10
        )
        self.cmd_q = command_queue
        self.telem_q = telemetry_queue
        self.landing_target_id = landing_target_id


    def _send(self, action: str, **kwargs):
        """Send a command to the MAVLink comm process via the command queue."""
        cmd = create_command(action, **kwargs)
        self.cmd_q.put(cmd)

    def _get_altitude(self) -> float:
        """Return the current altitude in meters (positive = above ground).

        In NED frame, pos_z_m is negative when the drone is above the origin.
        We negate it to get a positive altitude value.
        """
        return -self.telem.get("pos_z_m", 0.0)


    def vision_callback(self, msg: Point):
        """
        Callback function that is called when a new vision message is received.
        It publishes the landing target position to the specified topic.
        """
        if msg.z != self.landing_target_id:
            return

        self._send("send_landing_target", target=[msg.x, msg.y, self._get_altitude()])  # Get the latest telemetry data from the queue




