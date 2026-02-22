from restaurant_task.infrastructure.hsrb.robot.hsrb_robot_agent.infrastructure.laser_scan import LaserScanSubscriber

class DetectObstacleService:
    def __init__(self, laser_scan_subscriber: LaserScanSubscriber):
        self._laser_scan_subscriber = laser_scan_subscriber

    def execute(self, max_distance: float) -> bool:
        ranges = None
        while not ranges:
            ranges = self._laser_scan_subscriber.ranges
        front_space = ranges[len(ranges)//2]  # HUCK
        if front_space < max_distance:
            return True
        return False
