import math
import rclpy

from restaurant_task.infrastructure.hsrb.robot.hsrb_robot_agent.infrastructure.cmd_vel import CmdVelPublisher
from restaurant_task.infrastructure.hsrb.robot.hsrb_robot_agent.infrastructure.wheel_odom import WheelOdomSubscriber

class MoveService:
    def __init__(
            self,
            cmd_vel_publisher: CmdVelPublisher,
            wheel_odom_subscriber: WheelOdomSubscriber
        ) -> None:
        self._cmd_vel = cmd_vel_publisher
        self._wheel_odom = wheel_odom_subscriber

    def execute(self, x: float, y: float) -> None:
        self._move_x(x)
        self._move_y(y)

    def _move_x(self, meter: float, velosity: float = 0.05) -> None:
        x0, y0 = self._wheel_odom.x, self._wheel_odom.y

        while rclpy.ok():
            try:
                diff = self._diff_from(x0, y0)
            except ValueError:
                continue

            if diff >= abs(meter):
                self._cmd_vel.update(0.0, 0.0, 0.0)
                rclpy.spin_once(self)
                break

            self._cmd_vel.update((meter/abs(meter))*velosity, 0.0, 0.0)
            rclpy.spin_once(self)

    def _move_y(self, meter: float, velosity: float = 0.05) -> None:
        x0, y0 = self._wheel_odom.x, self._wheel_odom.y

        while rclpy.ok():
            try:
                diff = self._diff_from(x0, y0)
            except ValueError:
                continue

            if diff >= abs(meter):
                self._cmd_vel.update(0.0, 0.0, 0.0)
                rclpy.spin_once(self)
                break

            self._cmd_vel.update(0.0, (meter/abs(meter))*velosity, 0.0)
            rclpy.spin_once(self)

    def _diff_from(self, x0: float, y0: float) -> float:
        rclpy.spin_once(self)

        diff_x = self._wheel_odom.x - x0
        diff_y = self._wheel_odom.y - y0

        return math.sqrt((diff_x)**2 + (diff_y)**2)
