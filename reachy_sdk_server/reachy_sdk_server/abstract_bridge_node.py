from control_msgs.msg import DynamicJointState
import rclpy
from rclpy.node import Node
from threading import Event

from .utils import parse_reachy_config
from .components import Component, ComponentsHolder


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

        self.wait_for_setup()

    def wait_for_setup(self) -> None:
        while not self.got_first_state.is_set():
            rclpy.spin_once(self)

        self.setup_joint_state(self._first_state)
        self.joint_state_ready.set()

    # Orbita2D
    def get_all_orbita2ds(self):
        return self.components.get_by_type("orbita2d")

    # State updates
    def update_state(self, msg: DynamicJointState) -> None:
        if not self.got_first_state.is_set():
            self._first_state = msg
            self.got_first_state.set()
            return

        if not self.joint_state_ready.is_set():
            return

        for name, kv in zip(msg.joint_names, msg.interface_values):
            state = dict(zip(kv.interface_names, kv.values))
            self.components.get_by_name(name).update_state(state)

    def setup_joint_state(self, msg: DynamicJointState) -> None:
        for name, kv in zip(msg.joint_names, msg.interface_values):
            state = dict(zip(kv.interface_names, kv.values))
            self.logger.info(f"Adding component '{name}' with state '{state}'.")
            self.components.add_component(name, state=state, node_delegate=self)

        self.logger.info(
            f"Joint state setup (nb_joints={len(self.components.components)})."
        )
        for c in self.components.components:
            self.logger.info(f" - {c}")

    def get_state_for_component(self, component_name: str):
        return None
