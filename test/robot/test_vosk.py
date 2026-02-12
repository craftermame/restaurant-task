from restaurant_task.infrastructure.hsrb.robot.hsrb_robot_agent.infrastructure.vosk import vosk_client

async def test_vosk():
    for _ in range(5):
        print(await vosk_client.execute(5))
