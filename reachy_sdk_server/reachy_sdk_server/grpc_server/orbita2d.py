import math
from collections import namedtuple
from typing import Iterator

import grpc
import rclpy
from control_msgs.msg import DynamicJointState, InterfaceValue
from google.protobuf.empty_pb2 import Empty
from google.protobuf.wrappers_pb2 import BoolValue, FloatValue
from reachy2_sdk_api.component_pb2 import ComponentId, JointLimits, PIDGains
from reachy2_sdk_api.error_pb2 import Error
from reachy2_sdk_api.orbita2d_pb2 import (
    Float2d,
    Limits2d,
    ListOfOrbita2d,
    Orbita2d,
    Orbita2dCommand,
    Orbita2dField,
    Orbita2dsCommand,
    Orbita2dState,
    Orbita2dStateRequest,
    Orbita2dStatus,
    Orbita2dStreamStateRequest,
    PID2d,
    Pose2d,
    Vector2d,
)
from reachy2_sdk_api.orbita2d_pb2_grpc import add_Orbita2dServiceServicer_to_server

from ..abstract_bridge_node import AbstractBridgeNode
from ..components import Component
from ..utils import BOARD_STATUS, axis_from_str, endless_get_stream, extract_fields, get_current_timestamp

Orbita2dComponents = namedtuple("Orbita2dComponents", ["actuator", "axis1", "axis2", "raw_motor_1", "raw_motor_2"])


class Orbita2dServicer:
    default_fields = [
        Orbita2dField.PRESENT_POSITION,
        Orbita2dField.GOAL_POSITION,
        Orbita2dField.PRESENT_SPEED,
        Orbita2dField.PRESENT_LOAD,
        Orbita2dField.TEMPERATURE,
        Orbita2dField.JOINT_LIMITS,
        Orbita2dField.TORQUE_LIMIT,
        Orbita2dField.SPEED_LIMIT,
        Orbita2dField.PID,
        Orbita2dField.COMPLIANT,
    ]

    def __init__(
        self,
        bridge_node: AbstractBridgeNode,
        logger: rclpy.impl.rcutils_logger.RcutilsLogger,
    ) -> None:
        self.bridge_node = bridge_node
        self.logger = logger

    def register_to_server(self, server: grpc.Server):
        self.logger.info("Registering 'Orbita2dServiceServicer' to server.")
        add_Orbita2dServiceServicer_to_server(self, server)

    @classmethod
    def get_info(cls, orbita2d: Component) -> Orbita2d:
        return Orbita2d(
            id=ComponentId(
                id=orbita2d.id,
                name=orbita2d.name,
            ),
            axis_1=axis_from_str(orbita2d.extra["axis1"]),
            axis_2=axis_from_str(orbita2d.extra["axis2"]),
        )

    def GetAllOrbita2d(self, request: Empty, context: grpc.ServicerContext) -> ListOfOrbita2d:
        return ListOfOrbita2d(orbita2d=[self.get_info(o) for o in self.bridge_node.components.get_by_type("orbita2d")])

    # State
    def GetState(self, request: Orbita2dStateRequest, context: grpc.ServicerContext) -> Orbita2dState:
        orbita2d_components = self.get_orbita2d_components(request.id, context=context)
        state = extract_fields(Orbita2dField, request.fields, conversion_table, orbita2d_components)

        state["timestamp"] = get_current_timestamp(self.bridge_node)
        state["temperature"] = Float2d(motor_1=FloatValue(value=40.0), motor_2=FloatValue(value=40.0))
        state["joint_limits"] = Limits2d(
            axis_1=JointLimits(min=FloatValue(value=0.0), max=FloatValue(value=100.0)),
            axis_2=JointLimits(min=FloatValue(value=0.0), max=FloatValue(value=100.0)),
        )
        return Orbita2dState(**state)

    def StreamState(self, request: Orbita2dStreamStateRequest, context: grpc.ServicerContext) -> Iterator[Orbita2dState]:
        return endless_get_stream(
            self.GetState,
            request.req,
            context,
            1,
            # 1 / request.freq,
        )

    # Command
    def SendCommand(self, request: Orbita2dsCommand, context: grpc.ServicerContext) -> Empty:
        cmd = DynamicJointState()
        cmd.joint_names = []

        for req_cmd in request.cmd:
            if not req_cmd.HasField("id"):
                context.abort(grpc.StatusCode.INVALID_ARGUMENT, "Missing 'id' field.")

            orbita2d_components = self.get_orbita2d_components(req_cmd.id, context=context)

            if req_cmd.HasField("compliant"):
                cmd.joint_names.append(orbita2d_components.actuator.name)
                cmd.interface_values.append(
                    InterfaceValue(
                        interface_names=["torque"],
                        values=[not req_cmd.compliant.value],
                    )
                )

            if req_cmd.HasField("goal_position"):
                state = extract_fields(
                    Orbita2dField,
                    [Orbita2dField.GOAL_POSITION],
                    conversion_table,
                    orbita2d_components,
                )
                axis_1_value = (
                    req_cmd.goal_position.axis_1.value
                    if req_cmd.goal_position.HasField("axis_1")
                    else state["goal_position"].axis_1.value
                )
                axis_2_value = (
                    req_cmd.goal_position.axis_2.value
                    if req_cmd.goal_position.HasField("axis_2")
                    else state["goal_position"].axis_2.value
                )

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
                            values=[axis_1_value],
                        ),
                        InterfaceValue(
                            interface_names=["position"],
                            values=[axis_2_value],
                        ),
                    ]
                )

            raw_commands = []

            if req_cmd.HasField("speed_limit"):
                raw_commands.extend(
                    [
                        InterfaceValue(
                            interface_names=["speed_limit"],
                            values=[req_cmd.speed_limit.motor_1.value],
                        ),
                        InterfaceValue(
                            interface_names=["speed_limit"],
                            values=[req_cmd.speed_limit.motor_2.value],
                        ),
                    ]
                )

            if req_cmd.HasField("torque_limit"):
                raw_commands.extend(
                    [
                        InterfaceValue(
                            interface_names=["torque_limit"],
                            values=[req_cmd.torque_limit.motor_1.value],
                        ),
                        InterfaceValue(
                            interface_names=["torque_limit"],
                            values=[req_cmd.torque_limit.motor_2.value],
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
            self.bridge_node.publish_command(cmd)

        return Empty()

    def StreamCommand(self, request_stream: Iterator[Orbita2dsCommand], context: grpc.ServicerContext) -> Empty:
        for request in request_stream:
            self.SendCommand(request, context)
        return Empty()

    # Doctor
    def Audit(self, request: ComponentId, context: grpc.ServicerContext) -> Orbita2dStatus:
        orbita2d_components = self.get_orbita2d_components(request, context=context)
        return Orbita2dStatus(errors=[Error(details=str(BOARD_STATUS[orbita2d_components.actuator.state["errors"]]))])

    def HeartBeat(self, request: ComponentId, context: grpc.ServicerContext) -> Empty:
        return Empty()

    def Restart(self, request: ComponentId, context: grpc.ServicerContext) -> Empty:
        return Empty()

    # Setup utils
    def get_orbita2d_components(self, component_id: ComponentId, context: grpc.ServicerContext) -> Orbita2dComponents:
        if not hasattr(self, "_lazy_components"):
            self._lazy_components = {}

        components = self.bridge_node.components

        c = components.get_by_component_id(component_id)
        if c is None:
            context.abort(
                grpc.StatusCode.NOT_FOUND,
                f"Could not find component with id '{component_id}'.",
            )

        if c.type != "orbita2d":
            context.abort(
                grpc.StatusCode.INVALID_ARGUMENT,
                f"Component '{component_id}' is not an orbita2d.",
            )

        if c.id not in self._lazy_components:
            orbita2d = components.get_by_component_id(component_id)
            orbita2d_axis1 = components.get_by_name(f"{orbita2d.name}_{orbita2d.extra['axis1']}")
            orbita2d_axis2 = components.get_by_name(f"{orbita2d.name}_{orbita2d.extra['axis2']}")
            orbita2d_raw_motor_1 = components.get_by_name(f"{orbita2d.name}_raw_motor_1")
            orbita2d_raw_motor_2 = components.get_by_name(f"{orbita2d.name}_raw_motor_2")

            self._lazy_components[c.id] = Orbita2dComponents(
                orbita2d,
                orbita2d_axis1,
                orbita2d_axis2,
                orbita2d_raw_motor_1,
                orbita2d_raw_motor_2,
            )

        return self._lazy_components[c.id]


conversion_table = {
    "id": lambda o: ComponentId(id=o.actuator.id, name=o.actuator.name),
    "present_position": lambda o: Pose2d(
        axis_1=FloatValue(value=o.axis1.state["position"]),
        axis_2=FloatValue(value=o.axis2.state["position"]),
    ),
    "present_speed": lambda o: Vector2d(
        x=FloatValue(value=o.axis1.state["velocity"]),
        y=FloatValue(value=o.axis2.state["velocity"]),
    ),
    "present_load": lambda o: Vector2d(
        x=FloatValue(value=o.axis1.state["effort"]),
        y=FloatValue(value=o.axis2.state["effort"]),
    ),
    "compliant": lambda o: BoolValue(value=not o.actuator.state["torque"]),
    "goal_position": lambda o: Pose2d(
        axis_1=FloatValue(value=o.axis1.state["target_position"]),
        axis_2=FloatValue(value=o.axis2.state["target_position"]),
    ),
    "speed_limit": lambda o: Float2d(
        motor_1=(
            FloatValue(value=o.raw_motor_1.state["speed_limit"])
            if not math.isnan(o.raw_motor_1.state["speed_limit"])
            else FloatValue(value=100.0)
        ),
        motor_2=(
            FloatValue(value=o.raw_motor_2.state["speed_limit"])
            if not math.isnan(o.raw_motor_2.state["speed_limit"])
            else FloatValue(value=100.0)
        ),
    ),
    "torque_limit": lambda o: Float2d(
        motor_1=(
            FloatValue(value=o.raw_motor_1.state["torque_limit"])
            if not math.isnan(o.raw_motor_1.state["torque_limit"])
            else FloatValue(value=100.0)
        ),
        motor_2=(
            FloatValue(value=o.raw_motor_2.state["torque_limit"])
            if not math.isnan(o.raw_motor_2.state["torque_limit"])
            else FloatValue(value=100.0)
        ),
    ),
    "pid": lambda o: PID2d(
        motor_1=PIDGains(
            p=(
                FloatValue(value=o.raw_motor_1.state["p_gain"])
                if not math.isnan(o.raw_motor_1.state["p_gain"])
                else FloatValue(value=100.0)
            ),
            i=(
                FloatValue(value=o.raw_motor_1.state["i_gain"])
                if not math.isnan(o.raw_motor_1.state["i_gain"])
                else FloatValue(value=100.0)
            ),
            d=(
                FloatValue(value=o.raw_motor_1.state["d_gain"])
                if not math.isnan(o.raw_motor_1.state["d_gain"])
                else FloatValue(value=100.0)
            ),
        ),
        motor_2=PIDGains(
            p=(
                FloatValue(value=o.raw_motor_2.state["p_gain"])
                if not math.isnan(o.raw_motor_2.state["p_gain"])
                else FloatValue(value=100.0)
            ),
            i=(
                FloatValue(value=o.raw_motor_2.state["i_gain"])
                if not math.isnan(o.raw_motor_2.state["i_gain"])
                else FloatValue(value=100.0)
            ),
            d=(
                FloatValue(value=o.raw_motor_2.state["d_gain"])
                if not math.isnan(o.raw_motor_2.state["d_gain"])
                else FloatValue(value=100.0)
            ),
        ),
    ),
}
