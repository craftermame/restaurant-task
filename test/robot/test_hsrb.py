from restaurant_task.config import config

from restaurant_task.infrastructure.hsrb.robot.hsrb_robot_agent import hsrb

async def test_speak():
    await hsrb.speak("This is the sample sentence.")

async def test_ask():
    await hsrb.speak("Please just answer coffee for following question.")

    order = await hsrb.ask(config.message.ORDER_QUESTION)
    assert order == "coffee"

async def test_sleep():
    await hsrb.speak("I'll speak after 3 seconds.")
    await hsrb.sleep(3.0)
    await hsrb.speak("3 seconds passed.")
