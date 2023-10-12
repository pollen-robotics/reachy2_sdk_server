from collections import namedtuple
from control_msgs.msg import DynamicJointState, InterfaceValue
import grpc
import rclpy
from typing import Iterator


from ..abstract_bridge_node import AbstractBridgeNode
from ..utils import (
    axis_from_str,
    endless_get_stream,
    extract_fields,
    get_current_timestamp,
)

from google.protobuf.empty_pb2 import Empty
from google.protobuf.wrappers_pb2 import BoolValue

from reachy_sdk_api_v2.component_pb2 import ComponentId, PIDGains
from reachy_sdk_api_v2.orbita2d_pb2 import (
    PID2D,
    Pose2D,
    Float2D,
    ListOfOrbita2DInfo,
    Orbita2DCommand,
    Orbita2DField,
    Orbita2DInfo,
    Orbita2DState,
    Orbita2DStateRequest,
    Orbita2DStatus,
    Orbita2DStreamStateRequest,
    Vector2D,
)
from reachy_sdk_api_v2.orbita2d_pb2_grpc import add_Orbita2DServiceServicer_to_server


from ..components import Component


Orbita2DComponents = namedtuple(
    "Orbita2DComponents", ["actuator", "axis1", "axis2", "raw_motor_1", "raw_motor_2"]
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

    @classmethod
    def get_info(cls, orbita2d: Component) -> Orbita2DInfo:
        return Orbita2DInfo(
            id=ComponentId(
                id=orbita2d.id,
                name=orbita2d.name,
            ),
            axis_1=axis_from_str(orbita2d.extra["axis1"]),
            axis_2=axis_from_str(orbita2d.extra["axis2"]),
        )

    def GetAllOrbita2D(
        self, request: Empty, context: grpc.ServicerContext
    ) -> ListOfOrbita2DInfo:
        return ListOfOrbita2DInfo(
            info=[
                self.get_info(o)
                for o in self.bridge_node.components.get_by_type("orbita2d")
            ]
        )

    # State
    def GetState(
        self, request: Orbita2DStateRequest, context: grpc.ServicerContext
    ) -> Orbita2DState:
        orbita2d_components = self.get_orbita2d_components(request.id)

        state = extract_fields(
            Orbita2DField, request.fields, conversion_table, orbita2d_components
        )
        state["timestamp"] = get_current_timestamp(self.bridge_node)
        return Orbita2DState(**state)

    def StreamState(
        self, request: Orbita2DStreamStateRequest, context: grpc.ServicerContext
    ) -> Iterator[Orbita2DState]:
        return endless_get_stream(
            self.GetState,
            request.req,
            context,
            1 / request.freq,
        )

    # Command
    def SendCommand(
        self, request: Orbita2DCommand, context: grpc.ServicerContext
    ) -> Empty:
        self.logger.info(f"Received command: {request}")

        if not request.HasField("id"):
            context.abort(grpc.StatusCode.INVALID_ARGUMENT, "Missing 'id' field.")

        orbita2d_components = self.get_orbita2d_components(request.id)

        cmd = DynamicJointState()
        cmd.joint_names = []

        if request.HasField("compliant"):
            cmd.joint_names.append(orbita2d_components.actuator.name)
            cmd.interface_values.append(
                InterfaceValue(
                    interface_names=["torque"],
                    values=[not request.compliant.value],
                )
            )

        if request.HasField("goal_position"):
            cmd.joint_names.extend(
                [
                    orbita2d_components.axis1.name,
                    orbita2d_components.axis2.name,
                ]
            )
            cmd.interface_values.extend(
                [
                    InterfaceValue(
                        interface_names=["position"],
                        values=[request.goal_position.axis_1],
                    ),
                    InterfaceValue(
                        interface_names=["position"],
                        values=[request.goal_position.axis_2],
                    ),
                ]
            )

        raw_commands = []

        if request.HasField("speed_limit"):
            raw_commands.extend(
                [
                    InterfaceValue(
                        interface_names=["speed_limit"],
                        values=[request.speed_limit.axis_1],
                    ),
                    InterfaceValue(
                        interface_names=["speed_limit"],
                        values=[request.speed_limit.axis_2],
                    ),
                ]
            )

        if request.HasField("torque_limit"):
            raw_commands.extend(
                [
                    InterfaceValue(
                        interface_names=["torque_limit"],
                        values=[request.torque_limit.axis_1],
                    ),
                    InterfaceValue(
                        interface_names=["torque_limit"],
                        values=[request.torque_limit.axis_2],
                    ),
                ]
            )

        if raw_commands:
            cmd.joint_names.extend(
                [
                    orbita2d_components.raw_motor_1.name,
                    orbita2d_components.raw_motor_2.name,
                ]
            )
            cmd.interface_values.extend(raw_commands)

        if cmd.joint_names:
            self.logger.debug(f"Publishing command: {cmd}")
            self.bridge_node.publish_command(cmd)

        return Empty()

    def StreamCommand(
        self, request_stream: Iterator[Orbita2DCommand], context: grpc.ServicerContext
    ) -> Empty:
        for request in request_stream:
            self.SendCommand(request, context)
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
    def get_orbita2d_components(self, component_id: ComponentId) -> Orbita2DComponents:
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


conversion_table = {
    "name": lambda o: o.actuator.name,
    "id": lambda o: o.actuator.id,
    "present_position": lambda o: Pose2D(
        axis_1=o.axis1.state["position"], axis_2=o.axis2.state["position"]
    ),
    "present_speed": lambda o: Vector2D(
        x=o.axis1.state["velocity"], y=o.axis2.state["velocity"]
    ),
    "present_load": lambda o: Vector2D(
        x=o.axis1.state["effort"], y=o.axis2.state["effort"]
    ),
    "compliant": lambda o: BoolValue(value=not o.actuator.state["torque"]),
    "goal_position": lambda o: Pose2D(
        axis_1=o.axis1.state["target_position"],
        axis_2=o.axis2.state["target_position"],
    ),
    "speed_limit": lambda o: Float2D(
        motor_1=o.raw_motor_1.state["speed_limit"],
        motor_2=o.raw_motor_2.state["speed_limit"],
    ),
    "torque_limit": lambda o: Float2D(
        motor_1=o.raw_motor_1.state["torque_limit"],
        motor_2=o.raw_motor_2.state["torque_limit"],
    ),
    "pid": lambda o: PID2D(
        motor_1=PIDGains(
            p=o.raw_motor_1.state["p_gain"],
            i=o.raw_motor_1.state["i_gain"],
            d=o.raw_motor_1.state["d_gain"],
        ),
        motor_2=PIDGains(
            p=o.raw_motor_2.state["p_gain"],
            i=o.raw_motor_2.state["i_gain"],
            d=o.raw_motor_2.state["d_gain"],
        ),
    ),
}
