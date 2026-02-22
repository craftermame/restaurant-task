import time

from restaurant_task.config import config

from restaurant_task.infrastructure.hsrb.robot.hsrb_robot_agent import HSRBRobotAgent
from restaurant_task.application.robot.wait_door_open_service.wait_door_open_service import WaitDoorOpenCommand, WaitDoorOpenService
from restaurant_task.infrastructure.csvdb.item.csvdb_item_repository import CSVDBItemRepository
from restaurant_task.infrastructure.csvdb.physical_item.csvdb_physical_item_repository import CSVDBPhysicalItemRepository

hsrb = HSRBRobotAgent(
    CSVDBItemRepository(),
    CSVDBPhysicalItemRepository()
)

def test_speak():
    hsrb.speak("This is the sample sentence.")

def test_ask():
    hsrb.speak("Please just answer coffee for following question.")

    order = hsrb.ask(config.message.ORDER_QUESTION)
    assert "coffee" in order

def test_detect_obstacle():
    hsrb.speak("Please set a door.")
    time.sleep(5)
    WaitDoorOpenService(hsrb).execute(WaitDoorOpenCommand(0.2))

def test_move():
    hsrb.speak("I will move forward 20 centimeter.")
    time.sleep(2)
    hsrb.move(0.0, 0.2)

def test_hand():
    hsrb.speak("I will open the hand")
    hsrb.open_hand()
    time.sleep(2)
    hsrb.close_hand()
