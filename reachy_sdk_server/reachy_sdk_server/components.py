from collections import defaultdict

from reachy_sdk_api_v2.component_pb2 import ComponentId
import rclpy
from typing import List, Optional

from .utils import get_uid_from_name


# Should have at least "name", "id" and "type" keys.
class Component:
    def __init__(
        self, name: str, id: int, type: str, extra: dict, state: dict = {}
    ) -> None:
        self.name = name
        self.id = id
        self.type = type
        self.extra = extra
        self.state = state

    def __repr__(self) -> str:
        return f"Component(name={self.name}, id={self.id}, type={self.type}, extra={self.extra} state={self.state})"

    def update_state(self, new_state) -> None:
        self.state.update(new_state)


class ComponentsHolder:
    def __init__(self, config: dict) -> None:
        self.components = []
        self.by_name = {}
        self.by_id = {}
        self.by_type = defaultdict(list)

        self.config = config

    def add_component(
        self,
        name: str,
        id: Optional[int] = None,
        type: Optional[str] = None,
        extra: dict = {},
        state: dict = {},
        node_delegate: Optional[rclpy.node.Node] = None,
    ) -> None:
        if id is None:
            id = get_uid_from_name(name, node_delegate)

        if type is None:
            type = guess_component_type(name, self.config)

        if extra == {}:
            extra = add_extra_if_any(name, self.config)

        c = Component(name, id, type, extra, state)

        self.components.append(c)
        self.by_name[c.name] = c
        self.by_id[c.id] = c
        self.by_type[c.type].append(c)

        return c

    def get_by_component_id(self, component_id: ComponentId) -> Component:
        if component_id.id:
            return self.get_by_id(component_id.id)
        elif component_id.name:
            return self.get_by_name(component_id.name)
        else:
            raise ValueError(f"Invalid component_id: {component_id}")

    def get_by_type(self, component_type: str) -> List[Component]:
        return self.by_type[component_type]

    def get_by_name(self, component_name: str) -> Component:
        return self.by_name[component_name]

    def get_by_id(self, component_id: int) -> Component:
        return self.by_id[component_id]


def guess_component_type(name: str, config: dict) -> str:
    for part in config["reachy"].values():
        for actuator in part.values():
            if actuator["name"] == name:
                return actuator["actuator"]

    return "unknown"


def add_extra_if_any(name: str, config: dict) -> dict:
    for part in config["reachy"].values():
        for actuator in part.values():
            if actuator["name"] == name:
                return actuator

    return {}
