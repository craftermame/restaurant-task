import asyncio

from restaurant_task.infrastructure.hsrb.robot.hsrb_robot_agent import hsrb

async def test_hsrb_speak():
    await hsrb.speak("This is the sample sentence.")

if __name__ == "__main__":
    asyncio.run(test_hsrb_speak())
