import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection3DArray

from restaurant_task.domain.models.physical_item.physical_item import PhysicalItem, PhysicalItemIdentity, ItemId
from restaurant_task.domain.models.shared.coordinate import Coordinate
from restaurant_task.domain.models.item.i_item_repository import IItemRepository
from restaurant_task.domain.models.physical_item.i_physical_item_repository import IPhysicalItemRepository
from restaurant_task.infrastructure.hsrb.robot.hsrb_robot_agent import utils

class ItemPoseSerivce(Node):
    """
    Adjusts the position of the robot so that it can grasp the object.
    ### 物体の座標を決定する
    - `id == target_object`の`msg`を5回記録する．
    - `msg..size`のなかで，`size.x + size.y`が最小のときの，物体の座標を保持する (0.3, 0.1, z)  # TODO: ロジックを考える
    - 最適な`position.x`（つまり，物体との奥行きの距離）を見つける -> x_gap = 0.60 [m]
    - ロボットの中心横軸と，アームの中心横軸の差を測定する -> y_gap = 0.10 [m]
    - これは低レベルな関数: move_meter(0.3 - x_gap, 0.1 - y_gap)
    """
    @utils.with_rclpy
    def __init__(
            self,
            item_repository: IItemRepository,
            physical_item_repository: IPhysicalItemRepository
        ) -> None:
        super().__init__('pos_analysis')
        self.logger = self.get_logger()
        try:
            self.sub = self.create_subscription(
                Detection3DArray,
                '/yolo_ros/object_3d_poses',
                self.update_data,
                1
            )
        except TypeError:
            self.get_logger().error("Cannot create yolo subscription")
        self.item_repository = item_repository
        self.physical_item_repository = physical_item_repository

        self.object_data = {
            obj: {
                "min_bbox_size": (None, None, None),
                "position": (None, None, None),
                "count": 0
            } for obj in [item_id.value for item_id in self.item_repository.get_all_ids()]
        }

    def update_data(self, msg: Detection3DArray) -> None:
        """
        座標情報を取得して，更新します．
        同じ物体を何度か検出することで，バウンディングボックスの精度を高めます．
        """
        for detection in msg.detections:

            # データの受け取りと設定
            oid: str = detection.id  # Object ID
            bbox = detection.bbox.size
            position = detection.bbox.center.position

            # 初期データ
            if not self.object_data[oid]["count"]:
                self.object_data[oid]["count"] += 1
                self.object_data[oid]["min_bbox_size"] = bbox
                self.object_data[oid]["position"] = position
                continue

            # データの更新
            self.object_data[oid]["count"] += 1

            current_min_length = sum(vector3_to_tuple(self.object_data[oid]["min_bbox_size"]))
            length = sum(vector3_to_tuple(bbox))
            if length < current_min_length:  # バウンディングボックスの枠長さの総和が最小のものを登録する
                continue                     # ただし，カウントは増加させる

            self.object_data[oid]["min_bbox_size"] = bbox
            self.object_data[oid]["position"] = position

    def find_item(
            self,
            item_id: str,
            reliability: int = 5
        ) -> PhysicalItem:
        """
        数回記録して，そのなかでバウンディングボックスの週の長さが最小のものの座標を用いる．
        - object_name: ターゲットの物体名
        - reliability: 何回記録するか（デフォルトは 5）
        - 物体の三次元座標を返す．
        - 記録がなかった場合は，`(None, None, None)`を返す．
        """
        while rclpy.ok():
            rclpy.spin_once(self)
            if self.object_data[item_id]["count"] >= reliability:
                self.object_data[item_id]["count"] = 0  # HUCK
                v = vector3_to_tuple(self.object_data[item_id]["position"])

                physical_item = PhysicalItem(
                    PhysicalItemIdentity(ItemId(item_id)),
                    Coordinate(v[0], v[1], v[2])
                )

                self.physical_item_repository.save(physical_item)

                return physical_item

def vector3_to_tuple(vec3) -> tuple[float, float, float]:
    """Return tupled value"""
    return (vec3.x, vec3.y, vec3.z)
