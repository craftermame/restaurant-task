from rclpy.node import Node
from geometry_msgs.msg import Twist

from restaurant_task.infrastructure.hsrb.robot.hsrb_robot_agent import utils
from restaurant_task.infrastructure.hsrb.robot.hsrb_robot_agent.config import config

class CmdVelPublisher(Node):
    @utils.with_rclpy
    def __init__(self):
        super().__init__("cmd_vel")
        self._pub = self.create_publisher(Twist, config.msg.CMD_VEL, 10)
        self._timer = self.create_timer(0.01, self._timer_callback)

        self._vel_x = 0.0
        self._vel_y = 0.0
        self._angular = 0.0

    def update(
            self,
            vel_x: float = None,
            vel_y: float = None,
            angular: float = None
        ) -> None:
        self._vel_x = vel_x if vel_x else self._vel_x
        self._vel_y = vel_y if vel_y else self._vel_y
        self._angular = angular if angular else self._angular

    def _timer_callback(self):
        twist = Twist()
        twist.linear.x = self._vel_x
        twist.linear.y = self._vel_y
        twist.angular.z = self._angular

        self._pub.publish(twist)

cmd_vel = CmdVelPublisher()
