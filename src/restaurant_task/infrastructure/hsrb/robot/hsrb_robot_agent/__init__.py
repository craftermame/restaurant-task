import time

from restaurant_task.domain.models.robot.i_robot_agent import IRobotAgent

from restaurant_task.infrastructure.hsrb.robot.hsrb_robot_agent.client.supertonic import supertonic_client
from restaurant_task.infrastructure.hsrb.robot.hsrb_robot_agent.client.vosk import vosk_client

class HSRBRobotAgent(IRobotAgent):
    async def speak(self, sentence):
        await supertonic_client.execute(sentence)

    async def ask(self, question, duration_sec = 4):
        await self.speak(question)
        return await vosk_client.execute(duration_sec)

    async def sleep(self, duration_sec):
        time.sleep(duration_sec)

hsrb = HSRBRobotAgent()
