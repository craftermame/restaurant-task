import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from restaurant_task.infrastructure.hsrb.robot.hsrb_robot_agent import utils
from restaurant_task.infrastructure.hsrb.robot.hsrb_robot_agent.config import config

class LaserScanSubscriber(Node):
    @utils.with_rclpy
    def __init__(self):
        super().__init__("LiDAR")
        try:
            self.sub = self.create_subscription(
                LaserScan,
                config.msg.LASER_SCAN,
                self._callback,
                10
            )
        except TypeError:
            self.get_logger().error("Cannot create laser scan subscriprion")

        self._ranges: list[float] = []

    @property
    def ranges(self):
        return self._ranges

    def _callback(self, scan: LaserScan):
        rclpy.spin_once(self)
        self._ranges = list(scan.ranges)

laser_scan = LaserScanSubscriber()
