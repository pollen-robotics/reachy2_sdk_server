from collections import defaultdict
import rclpy
from typing import List

from .utils import get_uid_from_name

# Should have at least "name", "id" and "type" keys.
Component = dict


class ComponentsHolder:
    def __init__(self, components: List[Component]) -> None:
        self.components = components

        self.by_name = defaultdict(list)
        self.by_id = defaultdict(list)
        self.by_type = defaultdict(list)

        for c in self.components:
            self.by_name[c["name"]].append(c)
            self.by_id[c["id"]].append(c)
            self.by_type[c["actuator"]].append(c)

    @classmethod
    def from_config(cls, config: dict, node_delegate: rclpy.node.Node):
        components = []

        for part in config["reachy"].values():
            for actuator in part.values():
                actuator["id"] = get_uid_from_name(actuator["name"], node_delegate)
                components.append(actuator)

        return cls(components)

    def get_by_type(self, component_type: str) -> List[Component]:
        return self.by_type[component_type]

    def get_by_name(self, component_name: str) -> Component:
        return self.by_name[component_name]

    def get_by_id(self, component_id: int) -> Component:
        return self.by_id[component_id]
