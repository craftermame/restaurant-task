import sys
import asyncio

from restaurant_task.application.robot.wait_door_open_service.wait_door_open_service import WaitDoorOpenService
from restaurant_task.application.robot.navigate_to_spot_service.navigate_to_spot_service import NavigateToSpotService
from restaurant_task.application.robot.take_order_service.take_order_service import TakeOrderService
from restaurant_task.application.robot.grasp_item_service.grasp_item_service import GraspItemService
from restaurant_task.application.robot.serve_item_service.serve_item_service import ServeItemService
from restaurant_task.application.robot.restaurant_task_service.restaurant_task_service import RestaurantTaskCommand, RestaurantTaskService

from restaurant_task.infrastructure.hsrb.robot.hsrb_robot_agent import HSRBRobotAgent
from restaurant_task.infrastructure.csvdb.order.csvdb_order_repository import CSVDBOrderRepository
from restaurant_task.infrastructure.csvdb.item.csvdb_item_repository import CSVDBItemRepository

async def main():
    if len(sys.argv) < 2:
        print("Usage: python main.py <seat_number>")
        sys.exit(1)

    seat_number = int(sys.argv[1])

    robot_agent = HSRBRobotAgent()
    order_repository = CSVDBOrderRepository()
    item_repository = CSVDBItemRepository()
    wait_door_open_service = WaitDoorOpenService(robot_agent)
    navigate_to_spot_service = NavigateToSpotService(robot_agent)
    take_order_service = TakeOrderService(robot_agent, order_repository, item_repository)
    grasp_item_service = GraspItemService(robot_agent, item_repository)
    serve_item_service = ServeItemService(robot_agent)

    restaurant_task = RestaurantTaskService(
        robot_agent=robot_agent,
        order_repository=order_repository,
        wait_door_open_service=wait_door_open_service,
        navigate_to_spot_service=navigate_to_spot_service,
        take_order_service=take_order_service,
        grasp_item_service=grasp_item_service,
        serve_item_service=serve_item_service,
    )

    await restaurant_task.execute(RestaurantTaskCommand(seat_number))

if __name__ == "__main__":
    asyncio.run(main())
