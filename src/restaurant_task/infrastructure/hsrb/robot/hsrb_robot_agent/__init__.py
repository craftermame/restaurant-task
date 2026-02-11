import rclpy

from restaurant_task.domain.models.robot.i_robot_agent import IRobotAgent

rclpy.init()
from restaurant_task.infrastructure.hsrb.robot.hsrb_robot_agent.client.supertonic import supertonic_client


class HSRBRobotAgent(IRobotAgent):
    async def speak(self, sentence):
        await supertonic_client.execute(sentence)

hsrb = HSRBRobotAgent()
