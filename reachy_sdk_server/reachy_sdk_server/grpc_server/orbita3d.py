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
from reachy2_sdk_api.kinematics_pb2 import ExtEulerAngles, Rotation3d
from reachy2_sdk_api.orbita3d_pb2 import (
    Float3d,
    Limits3d,
    ListOfOrbita3d,
    Orbita3d,
    Orbita3dCommand,
    Orbita3dField,
    Orbita3dGoal,
    Orbita3dsCommand,
    Orbita3dState,
    Orbita3dStateRequest,
    Orbita3dStatus,
    Orbita3dStreamStateRequest,
    PID3d,
    Vector3d,
)
from reachy2_sdk_api.orbita3d_pb2_grpc import add_Orbita3dServiceServicer_to_server

from ..abstract_bridge_node import AbstractBridgeNode
from ..components import Component
from ..conversion import rotation3d_as_extrinsinc_euler_angles
from ..utils import BOARD_STATUS, endless_get_stream, extract_fields, get_current_timestamp

Orbita3dComponents = namedtuple(
    "Orbita3dComponents",
    ["actuator", "roll", "pitch", "yaw", "raw_motor_1", "raw_motor_2", "raw_motor_3"],
)


class Orbita3dServicer:
    default_fields = [
        Orbita3dField.PRESENT_POSITION,
        Orbita3dField.GOAL_POSITION,
        Orbita3dField.PRESENT_SPEED,
        Orbita3dField.PRESENT_LOAD,
        Orbita3dField.TEMPERATURE,
        Orbita3dField.JOINT_LIMITS,
        Orbita3dField.TORQUE_LIMIT,
        Orbita3dField.SPEED_LIMIT,
        Orbita3dField.PID,
        Orbita3dField.COMPLIANT,
    ]

    def __init__(
        self,
        bridge_node: AbstractBridgeNode,
        logger: rclpy.impl.rcutils_logger.RcutilsLogger,
    ) -> None:
        self.bridge_node = bridge_node
        self.logger = logger

    def register_to_server(self, server: grpc.Server):
        self.logger.info("Registering 'Orbita3dServiceServicer' to server.")
        add_Orbita3dServiceServicer_to_server(self, server)

    @classmethod
    def get_info(cls, orbita3d: Component) -> Orbita3d:
        return Orbita3d(
            id=ComponentId(
                id=orbita3d.id,
                name=orbita3d.name,
            ),
        )

    def GetAllOrbita3d(self, request: Empty, context: grpc.ServicerContext) -> ListOfOrbita3d:
        return ListOfOrbita3d(info=[self.get_info(o) for o in self.bridge_node.components.get_by_type("orbita3d")])

    def GetState(self, request: Orbita3dStateRequest, context: grpc.ServicerContext) -> Orbita3dState:
        orbita3d_components = self.get_orbita3d_components(request.id, context=context)

        state = extract_fields(Orbita3dField, request.fields, conversion_table, orbita3d_components)
        state["timestamp"] = get_current_timestamp(self.bridge_node)
        state["temperature"] = Float3d(
            motor_1=FloatValue(value=40.0),
            motor_2=FloatValue(value=40.0),
            motor_3=FloatValue(value=40.0),
        )
        state["joint_limits"] = Limits3d(
            roll=JointLimits(min=FloatValue(value=0.0), max=FloatValue(value=100.0)),
            pitch=JointLimits(min=FloatValue(value=0.0), max=FloatValue(value=100.0)),
            yaw=JointLimits(min=FloatValue(value=0.0), max=FloatValue(value=100.0)),
        )
        return Orbita3dState(**state)

    def GoToOrientation(self, request: Orbita3dGoal, context: grpc.ServicerContext) -> Empty:
        return Empty()

    def StreamState(self, request: Orbita3dStreamStateRequest, context: grpc.ServicerContext) -> Iterator[Orbita3dState]:
        return endless_get_stream(
            self.GetState,
            request.req,
            context,
            # 1 / request.freq,
            1,
        )

    def build_command(self, request: Orbita3dCommand, context: grpc.ServicerContext) -> DynamicJointState:
        cmd = DynamicJointState()
        cmd.joint_names = []

        for cmd_req in request.cmd:
            if not cmd_req.HasField("id"):
                context.abort(grpc.StatusCode.INVALID_ARGUMENT, "Missing 'id' field.")

            orbita3d_components = self.get_orbita3d_components(cmd_req.id, context=context)

            if cmd_req.HasField("compliant"):
                cmd.joint_names.append(orbita3d_components.actuator.name)
                cmd.interface_values.append(
                    InterfaceValue(
                        interface_names=["torque"],
                        values=[not cmd_req.compliant.value],
                    )
                )
            if cmd_req.HasField("goal_position"):
                if cmd_req.goal_position.HasField("rpy"):
                    state = extract_fields(
                        Orbita3dField,
                        [Orbita3dField.GOAL_POSITION],
                        conversion_table,
                        orbita3d_components,
                    )

                    roll_value = (
                        cmd_req.goal_position.rpy.roll
                        if cmd_req.goal_position.rpy.HasField("roll")
                        else state["goal_position"].rpy.roll
                    )
                    pitch_value = (
                        cmd_req.goal_position.rpy.pitch
                        if cmd_req.goal_position.rpy.HasField("pitch")
                        else state["goal_position"].rpy.pitch
                    )
                    yaw_value = (
                        cmd_req.goal_position.rpy.yaw
                        if cmd_req.goal_position.rpy.HasField("yaw")
                        else state["goal_position"].rpy.yaw
                    )

                    cmd_req = Orbita3dCommand(
                        goal_position=Rotation3d(rpy=ExtEulerAngles(roll=roll_value, pitch=pitch_value, yaw=yaw_value))
                    )

                roll, pitch, yaw = rotation3d_as_extrinsinc_euler_angles(cmd_req.goal_position)
                cmd.joint_names.extend(
                    [
                        orbita3d_components.roll.name,
                        orbita3d_components.pitch.name,
                        orbita3d_components.yaw.name,
                    ]
                )
                cmd.interface_values.extend(
                    [
                        InterfaceValue(
                            interface_names=["position"],
                            values=[roll],
                        ),
                        InterfaceValue(
                            interface_names=["position"],
                            values=[pitch],
                        ),
                        InterfaceValue(
                            interface_names=["position"],
                            values=[yaw],
                        ),
                    ]
                )

            raw_commands = []

            if cmd_req.HasField("speed_limit"):
                raw_commands.extend(
                    [
                        InterfaceValue(
                            interface_names=["speed_limit"],
                            values=[cmd_req.speed_limit.motor_1.value],
                        ),
                        InterfaceValue(
                            interface_names=["speed_limit"],
                            values=[cmd_req.speed_limit.motor_2.value],
                        ),
                        InterfaceValue(
                            interface_names=["speed_limit"],
                            values=[cmd_req.speed_limit.motor_3.value],
                        ),
                    ]
                )
            if cmd_req.HasField("torque_limit"):
                raw_commands.extend(
                    [
                        InterfaceValue(
                            interface_names=["torque_limit"],
                            values=[cmd_req.torque_limit.motor_1.value],
                        ),
                        InterfaceValue(
                            interface_names=["torque_limit"],
                            values=[cmd_req.torque_limit.motor_2.value],
                        ),
                        InterfaceValue(
                            interface_names=["torque_limit"],
                            values=[cmd_req.torque_limit.motor_3.value],
                        ),
                    ]
                )

            if raw_commands:
                cmd.joint_names.extend(
                    [
                        orbita3d_components.raw_motor_1.name,
                        orbita3d_components.raw_motor_2.name,
                        orbita3d_components.raw_motor_3.name,
                    ]
                )
                cmd.interface_values.extend(raw_commands)

        return cmd

    def SendCommand(self, request: Orbita3dsCommand, context: grpc.ServicerContext) -> Empty:
        cmd = self.build_command(request, context)

        if cmd.joint_names:
            self.bridge_node.publish_command(cmd)

        return Empty()

    def StreamCommand(self, request_stream: Iterator[Orbita3dCommand], context: grpc.ServicerContext) -> Empty:
        for request in request_stream:
            self.SendCommand(request, context)
        return Empty()

    def Audit(self, request: ComponentId, context: grpc.ServicerContext) -> Orbita3dStatus:
        orbita3d_components = self.get_orbita3d_components(request, context=context)
        return Orbita3dStatus(errors=[Error(details=str(BOARD_STATUS[orbita3d_components.actuator.state["errors"]]))])

    def HeartBeat(self, request: ComponentId, context: grpc.ServicerContext) -> Empty:
        return Empty()

    def Restart(self, request: ComponentId, context: grpc.ServicerContext) -> Empty:
        return Empty()

    def get_orbita3d_components(self, component_id: ComponentId, context: grpc.ServicerContext) -> Orbita3dComponents:
        if not hasattr(self, "_lazy_components"):
            self._lazy_components = {}

        components = self.bridge_node.components

        c = components.get_by_component_id(component_id)
        if c is None:
            context.abort(
                grpc.StatusCode.NOT_FOUND,
                f"Could not find component with id '{component_id}'.",
            )

        if c.type != "orbita3d":
            context.abort(
                grpc.StatusCode.INVALID_ARGUMENT,
                f"Component '{component_id}' is not an orbita3d.",
            )

        if c.id not in self._lazy_components:
            orbita3d = components.get_by_component_id(component_id)
            orbita3d_roll = components.get_by_name(f"{orbita3d.name}_roll")
            orbita3d_pitch = components.get_by_name(f"{orbita3d.name}_pitch")
            orbita3d_yaw = components.get_by_name(f"{orbita3d.name}_yaw")
            orbita3d_raw_motor_1 = components.get_by_name(f"{orbita3d.name}_raw_motor_1")
            orbita3d_raw_motor_2 = components.get_by_name(f"{orbita3d.name}_raw_motor_2")
            orbita3d_raw_motor_3 = components.get_by_name(f"{orbita3d.name}_raw_motor_3")

            self._lazy_components[c.id] = Orbita3dComponents(
                orbita3d,
                orbita3d_roll,
                orbita3d_pitch,
                orbita3d_yaw,
                orbita3d_raw_motor_1,
                orbita3d_raw_motor_2,
                orbita3d_raw_motor_3,
            )

        return self._lazy_components[c.id]


conversion_table = {
    "id": lambda o: ComponentId(id=o.actuator.id, name=o.actuator.name),
    "present_position": lambda o: Rotation3d(
        rpy=ExtEulerAngles(
            roll=FloatValue(value=o.roll.state["position"]),
            pitch=FloatValue(value=o.pitch.state["position"]),
            yaw=FloatValue(value=o.yaw.state["position"]),
        ),
    ),
    "present_speed": lambda o: Vector3d(
        x=FloatValue(value=o.roll.state["velocity"]),
        y=FloatValue(value=o.pitch.state["velocity"]),
        z=FloatValue(value=o.yaw.state["velocity"]),
    ),
    "present_load": lambda o: Vector3d(
        x=FloatValue(value=o.roll.state["effort"]),
        y=FloatValue(value=o.pitch.state["effort"]),
        z=FloatValue(value=o.yaw.state["effort"]),
    ),
    "compliant": lambda o: BoolValue(value=not o.actuator.state["torque"]),
    "goal_position": lambda o: Rotation3d(
        rpy=ExtEulerAngles(
            roll=FloatValue(value=o.roll.state["target_position"]),
            pitch=FloatValue(value=o.pitch.state["target_position"]),
            yaw=FloatValue(value=o.yaw.state["target_position"]),
        ),
    ),
    "speed_limit": lambda o: Float3d(
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
        motor_3=(
            FloatValue(value=o.raw_motor_3.state["speed_limit"])
            if not math.isnan(o.raw_motor_3.state["speed_limit"])
            else FloatValue(value=100.0)
        ),
    ),
    "torque_limit": lambda o: Float3d(
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
        motor_3=(
            FloatValue(value=o.raw_motor_3.state["torque_limit"])
            if not math.isnan(o.raw_motor_3.state["torque_limit"])
            else FloatValue(value=100.0)
        ),
    ),
    "pid": lambda o: PID3d(
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
        motor_3=PIDGains(
            p=(
                FloatValue(value=o.raw_motor_3.state["p_gain"])
                if not math.isnan(o.raw_motor_3.state["p_gain"])
                else FloatValue(value=100.0)
            ),
            i=(
                FloatValue(value=o.raw_motor_3.state["i_gain"])
                if not math.isnan(o.raw_motor_3.state["i_gain"])
                else FloatValue(value=100.0)
            ),
            d=(
                FloatValue(value=o.raw_motor_3.state["d_gain"])
                if not math.isnan(o.raw_motor_3.state["d_gain"])
                else FloatValue(value=100.0)
            ),
        ),
    ),
}
