from collections import namedtuple
from typing import Iterator, List
import grpc
from reachy_sdk_api_v2.component_pb2 import ComponentId, PIDGains
import rclpy


from ..abstract_bridge_node import AbstractBridgeNode
from ..components import Component
from ..utils import axis_from_str

from google.protobuf.empty_pb2 import Empty
from google.protobuf.wrappers_pb2 import BoolValue

from reachy_sdk_api_v2.orbita2d_pb2 import (
    PID2D,
    Float2D,
    ListOfOrbita2DInfo,
    Orbita2DCommand,
    Orbita2DField,
    Orbita2DInfo,
    Orbita2DState,
    Orbita2DStateRequest,
    Orbita2DStatus,
    Orbita2DStreamStateRequest,
)
from reachy_sdk_api_v2.orbita2d_pb2_grpc import add_Orbita2DServiceServicer_to_server


Orbita2DComponents = namedtuple(
    "Orbita2dComponents", ["actuator", "axis1", "axis2", "raw_motor_1", "raw_motor_2"]
)


class Orbita2dServicer:
    def __init__(
        self,
        bridge_node: AbstractBridgeNode,
        logger: rclpy.impl.rcutils_logger.RcutilsLogger,
    ) -> None:
        self.bridge_node = bridge_node
        self.logger = logger

    def register_to_server(self, server: grpc.Server):
        self.logger.info("Registering 'Orbita2dServiceServicer' to server.")
        add_Orbita2DServiceServicer_to_server(self, server)

    def GetAllOrbita2D(
        self, request: Empty, context: grpc.ServicerContext
    ) -> ListOfOrbita2DInfo:
        orbita2d = self.bridge_node.get_all_orbita2ds()

        infos = ListOfOrbita2DInfo(
            info=[
                Orbita2DInfo(
                    id=ComponentId(
                        id=o.id,
                        name=o.name,
                    ),
                    axis_1=axis_from_str(o.extra["axis1"]),
                    axis_2=axis_from_str(o.extra["axis2"]),
                )
                for o in orbita2d
            ]
        )
        return infos

    # State
    def GetState(
        self, request: Orbita2DStateRequest, context: grpc.ServicerContext
    ) -> Orbita2DState:
        orbita2d_components = self.get_orbita_components(request.id)

        state = extract_fields(orbita2d_components, request.fields)
        return Orbita2DState(**state)

    def StreamState(
        self, request: Orbita2DStreamStateRequest, context: grpc.ServicerContext
    ) -> Iterator[Orbita2DState]:
        yield Orbita2DState()

    # Command
    def SendCommand(
        self, request: Orbita2DCommand, context: grpc.ServicerContext
    ) -> Empty:
        return Empty()

    def StreamCommand(
        self, request_stream: Iterator[Orbita2DCommand], context: grpc.ServicerContext
    ) -> Empty:
        return Empty()

    # Doctor
    def Audit(
        self, request: ComponentId, context: grpc.ServicerContext
    ) -> Orbita2DStatus:
        return Orbita2DStatus()

    def HeartBeat(self, request: ComponentId, context: grpc.ServicerContext) -> Empty:
        return Empty()

    def Restart(self, request: ComponentId, context: grpc.ServicerContext) -> Empty:
        return Empty()

    # Setup utils
    def get_orbita_components(self, component_id: ComponentId) -> Orbita2DComponents:
        if not hasattr(self, "_lazy_components"):
            self._lazy_components = {}

        components = self.bridge_node.components

        if component_id.id not in self._lazy_components:
            orbita2d = components.get_by_component_id(component_id)
            orbita2d_axis1 = components.get_by_name(
                f"{orbita2d.name}_{orbita2d.extra['axis1']}"
            )
            orbita2d_axis2 = components.get_by_name(
                f"{orbita2d.name}_{orbita2d.extra['axis2']}"
            )
            orbita2d_raw_motor_1 = components.get_by_name(
                f"{orbita2d.name}_raw_motor_1"
            )
            orbita2d_raw_motor_2 = components.get_by_name(
                f"{orbita2d.name}_raw_motor_2"
            )

            self._lazy_components[component_id.id] = Orbita2DComponents(
                orbita2d,
                orbita2d_axis1,
                orbita2d_axis2,
                orbita2d_raw_motor_1,
                orbita2d_raw_motor_2,
            )

        return self._lazy_components[component_id.id]


def cleanup_fields(fields):
    if Orbita2DField.NONE in fields:
        cleanup_fields = []
    elif Orbita2DField.ALL in fields:
        cleanup_fields = list(Orbita2DField.keys())
        cleanup_fields.remove("ALL")
        cleanup_fields.remove("NONE")
    else:
        cleanup_fields = [
            Orbita2DField.DESCRIPTOR.values_by_number[field].name for field in fields
        ]

    cleanup_fields = [f.lower() for f in cleanup_fields]

    return cleanup_fields


def extract_fields(o: Orbita2DComponents, fields) -> dict:
    grpc_state = {}

    for f in cleanup_fields(fields):
        if f not in conversion_table:
            continue
        grpc_state[f] = conversion_table[f](o)

    return grpc_state


conversion_table = {
    "name": lambda o: o.actuator.name,
    "id": lambda o: o.actuator.id,
    "present_position": lambda o: Float2D(
        axis_1=o.axis1.state["position"], axis_2=o.axis2.state["position"]
    ),
    "present_speed": lambda o: Float2D(
        axis_1=o.axis1.state["velocity"], axis_2=o.axis2.state["velocity"]
    ),
    "present_load": lambda o: Float2D(
        axis_1=o.axis1.state["effort"], axis_2=o.axis2.state["effort"]
    ),
    "temperature": lambda o: Float2D(axis_1=37.5, axis_2=37.5),
    "compliant": lambda o: BoolValue(value=not o.actuator.state["torque"]),
    # "goal_position": lambda o: Float2D(
    #     axis_1=o.state["goal_position"],
    #     axis_2=o.state["goal_position"],
    # ),
    "speed_limit": lambda o: Float2D(
        axis_1=o.raw_motor_1.state["speed_limit"],
        axis_2=o.raw_motor_2.state["speed_limit"],
    ),
    "torque_limit": lambda o: Float2D(
        axis_1=o.raw_motor_1.state["torque_limit"],
        axis_2=o.raw_motor_2.state["torque_limit"],
    ),
    "pid": lambda o: PID2D(
        gains_axis_1=PIDGains(
            p=o.raw_motor_1.state["p_gain"],
            i=o.raw_motor_1.state["i_gain"],
            d=o.raw_motor_1.state["d_gain"],
        ),
        gains_axis_2=PIDGains(
            p=o.raw_motor_2.state["p_gain"],
            i=o.raw_motor_2.state["i_gain"],
            d=o.raw_motor_2.state["d_gain"],
        ),
    ),
}
