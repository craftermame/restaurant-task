from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from nav_msgs.msg import Odometry

from restaurant_task.infrastructure.hsrb.robot.hsrb_robot_agent import utils
from restaurant_task.infrastructure.hsrb.robot.hsrb_robot_agent.config import config

class WheelOdomSubscriber(Node):
    @utils.with_rclpy
    def __init__(self):
        super().__init__("wheel_odom")
        try:
            self.create_subscription(
                Odometry,
                config.msg.WHEEL_ODOM,
                self._subscription_callback,
                QoSProfile(
                    depth=10,
                    reliability=QoSReliabilityPolicy.BEST_EFFORT
                )
            )
        except TypeError:
            self.get_logger().error("Cannot create wheel odom subscription.")

        self._x = 0.0
        self._y = 0.0

    @property
    def x(self) -> float:
        return self._x

    @property
    def y(self) -> float:
        return self._y

    def _subscription_callback(self, odom: Odometry):
        self._x = odom.pose.pose.position.x
        self._y = odom.pose.pose.position.y

wheel_odom = WheelOdomSubscriber()
