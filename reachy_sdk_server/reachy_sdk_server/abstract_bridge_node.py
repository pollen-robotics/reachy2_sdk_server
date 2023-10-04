from rclpy.node import Node


from .utils import parse_reachy_config
from .components import ComponentsHolder


class AbstractBridgeNode(Node):
    def __init__(self, reachy_config_path: str = None) -> None:
        super().__init__(node_name="reachy_abstract_bridge_node")

        self.logger = self.get_logger()

        self.config = parse_reachy_config(reachy_config_path)
        self.logger.debug(f"Reachy config: {self.config}")

        self.components = ComponentsHolder.from_config(self.config, node_delegate=self)

    # Orbita2D
    def get_all_orbita2ds(self):
        return self.components.get_by_type("orbita2d")
