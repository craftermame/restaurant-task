import asyncio

from restaurant_task.infrastructure.hsrb.robot.hsrb_robot_agent import hsrb

async def test_hsrb_speak():
    raw_order = await hsrb.ask("What's your order?")
    print(raw_order)

if __name__ == "__main__":
    asyncio.run(test_hsrb_speak())
