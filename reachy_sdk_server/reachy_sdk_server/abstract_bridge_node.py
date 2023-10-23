from control_msgs.msg import DynamicJointState
from geometry_msgs.msg import Pose, PoseStamped
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from threading import Event, Lock
from typing import Tuple

from pollen_msgs.srv import GetForwardKinematics, GetInverseKinematics

from reachy_sdk_api_v2.component_pb2 import ComponentId
from reachy_sdk_api_v2.part_pb2 import PartId

from .components import ComponentsHolder
from .conversion import matrix_to_pose, pose_to_matrix
from .parts import PartsHolder
from .utils import parse_reachy_config


class AbstractBridgeNode(Node):
    def __init__(self, reachy_config_path: str = None) -> None:
        super().__init__(node_name="reachy_abstract_bridge_node")

        self.logger = self.get_logger()

        self.config = parse_reachy_config(reachy_config_path)
        self.components = ComponentsHolder(self.config)

        self.got_first_state = Event()
        self.joint_state_ready = Event()

        self.create_subscription(
            msg_type=DynamicJointState,
            topic="/dynamic_joint_states",
            callback=self.update_state,
            qos_profile=10,
        )

        self.command_pub_lock = Lock()
        self.joint_command_pub = self.create_publisher(
            msg_type=DynamicJointState,
            topic="/dynamic_joint_commands",
            qos_profile=10,
        )

        self.wait_for_setup()
        self.logger.info(
            f"Joint state setup (nb_joints={len(self.components.components)})."
        )
        for c in self.components.components:
            self.logger.info(f"\t - {c}")

        # Now that we have components, setup parts
        self.parts = PartsHolder(self.logger, self.config, self.components)

        # Register to services for kinematics
        self.setup_kinematics()

        self.create_subscription(
            msg_type=JointState,
            topic="/joint_commands",
            callback=self.update_command,
            qos_profile=10,
        )

    def wait_for_setup(self) -> None:
        # Wait for a first /dynamic_joint_state message to get a list of all joints
        while not self.got_first_state.is_set():
            rclpy.spin_once(self)

        # call service to get all value for each joint
        for name in self.joint_names:
            self.components.add_component(name, node_delegate=self)

        self.joint_state_ready.set()

    # State updates
    def update_state(self, msg: DynamicJointState) -> None:
        if not self.got_first_state.is_set():
            self.joint_names = msg.joint_names
            self.got_first_state.set()
            return

        if not self.joint_state_ready.is_set():
            return

        for name, kv in zip(msg.joint_names, msg.interface_values):
            state = dict(zip(kv.interface_names, kv.values))
            self.components.get_by_name(name).update_state(state)

    # Command updates
    def update_command(self, msg: JointState) -> None:
        for name, target in zip(msg.name, msg.position):
            self.components.get_by_name(name).update_command(
                {"target_position": target}
            )

    def publish_command(self, msg: DynamicJointState) -> None:
        with self.command_pub_lock:
            self.joint_command_pub.publish(msg)

    # Misc utils
    def get_component(self, component_id: ComponentId) -> dict:
        return self.components.get_by_component_id(component_id)

    # Kinematics
    def setup_kinematics(self):
        # Register to forward/inverse kinematics ROS services
        # And to /{side}_arm/target_pose
        self.forward_kinematics_clients = {}
        self.inverse_kinematics_clients = {}
        self.target_pose_pubs = {}

        self.command_target_pub_lock = Lock()

        for part in self.parts:
            c = self.create_client(
                srv_type=GetForwardKinematics,
                srv_name=f"/{part.name}/forward_kinematics",
            )
            self.logger.info(f"Subscribing for service '{c.srv_name}'...")
            c.wait_for_service()
            self.forward_kinematics_clients[part.id] = c

            c = self.create_client(
                srv_type=GetInverseKinematics,
                srv_name=f"/{part.name}/inverse_kinematics",
            )
            self.logger.info(f"Subscribing for service '{c.srv_name}'...")
            c.wait_for_service()
            self.inverse_kinematics_clients[part.id] = c

            self.target_pose_pubs[part.id] = self.create_publisher(
                msg_type=PoseStamped,
                topic=f"/{part.name}/target_pose",
                qos_profile=10,
            )
            self.logger.info(
                f"Publisher to topic '{self.target_pose_pubs[part.id].topic_name}' ready."
            )

    def compute_forward(
        self, id: PartId, joint_position: JointState
    ) -> Tuple[bool, np.array]:
        id = self.parts.get_by_part_id(id).id

        req = GetForwardKinematics.Request()
        req.joint_position = joint_position

        resp = self.forward_kinematics_clients[id].call(req)

        if resp.success:
            return True, pose_to_matrix(resp.pose)
        else:
            return False, None

    def compute_inverse(
        self, id: PartId, target: np.array, q0: JointState
    ) -> Tuple[bool, JointState]:
        id = self.parts.get_by_part_id(id).id

        req = GetInverseKinematics.Request()
        req.pose = matrix_to_pose(target)
        req.q0 = q0

        resp = self.inverse_kinematics_clients[id].call(req)
        return resp.success, resp.joint_position

    def publish_target_pose(self, id: PartId, pose: Pose) -> None:
        id = self.parts.get_by_part_id(id).id

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose = pose

        with self.command_target_pub_lock:
            self.target_pose_pubs[id].publish(msg)
