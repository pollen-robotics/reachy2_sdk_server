from control_msgs.msg import DynamicJointState
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from threading import Event, Lock

from reachy_sdk_api_v2.component_pb2 import ComponentId

from .utils import parse_reachy_config
from .components import ComponentsHolder


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

        self.logger.info(
            f"Joint state setup (nb_joints={len(self.components.components)})."
        )
        for c in self.components.components:
            self.logger.info(f"\t - {c}")

        self.joint_state_ready.set()

    # Orbita2D
    def get_all_orbita2ds(self):
        return self.components.get_by_type("orbita2d")

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
