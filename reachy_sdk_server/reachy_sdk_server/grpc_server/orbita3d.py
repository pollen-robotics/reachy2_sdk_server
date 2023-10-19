from collections import namedtuple
from control_msgs.msg import DynamicJointState, InterfaceValue
import grpc
import rclpy
from typing import Iterator

from google.protobuf.empty_pb2 import Empty
from google.protobuf.wrappers_pb2 import BoolValue, FloatValue

from reachy_sdk_api_v2.component_pb2 import ComponentId, PIDGains
from reachy_sdk_api_v2.kinematics_pb2 import ExtEulerAngles, Rotation3D
from reachy_sdk_api_v2.orbita3d_pb2 import (
    PID3D,
    Float3D,
    ListOfOrbita3D,
    Orbita3DCommand,
    Orbita3DsCommand,
    Orbita3DField,
    Orbita3D,
    Orbita3DState,
    Orbita3DStateRequest,
    Orbita3DStatus,
    Orbita3DStreamStateRequest,
    Vector3D,
)
from reachy_sdk_api_v2.orbita3d_pb2_grpc import add_Orbita3DServiceServicer_to_server

from ..abstract_bridge_node import AbstractBridgeNode
from ..conversion import rotation3d_as_extrinsinc_euler_angles
from ..components import Component
from ..utils import (
    endless_get_stream,
    extract_fields,
    get_current_timestamp,
)


Orbita3DComponents = namedtuple(
    "Orbita3DComponents",
    ["actuator", "roll", "pitch", "yaw", "raw_motor_1", "raw_motor_2", "raw_motor_3"],
)


class Orbita3dServicer:
    default_fields = [
        Orbita3DField.PRESENT_POSITION,
        Orbita3DField.GOAL_POSITION,
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
        add_Orbita3DServiceServicer_to_server(self, server)

    @classmethod
    def get_info(cls, orbita3d: Component) -> Orbita3D:
        return Orbita3D(
            id=ComponentId(
                id=orbita3d.id,
                name=orbita3d.name,
            ),
        )

    def GetAllOrbita3D(
        self, request: Empty, context: grpc.ServicerContext
    ) -> ListOfOrbita3D:
        return ListOfOrbita3D(
            info=[
                self.get_info(o)
                for o in self.bridge_node.components.get_by_type("orbita3d")
            ]
        )

    def GetState(
        self, request: Orbita3DStateRequest, context: grpc.ServicerContext
    ) -> Orbita3DState:
        orbita2d_components = self.get_orbita3d_components(request.id, context=context)

        state = extract_fields(
            Orbita3DField, request.fields, conversion_table, orbita2d_components
        )
        state["timestamp"] = get_current_timestamp(self.bridge_node)
        return Orbita3DState(**state)

    def StreamState(
        self, request: Orbita3DStreamStateRequest, context: grpc.ServicerContext
    ) -> Iterator[Orbita3DState]:
        return endless_get_stream(
            self.GetState,
            request.req,
            context,
            1 / request.freq,
        )

    def SendCommand(
        self, request: Orbita3DsCommand, context: grpc.ServicerContext
    ) -> Empty:
        self.logger.info(f"Received command: {request}")

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
                roll, pitch, yaw = rotation3d_as_extrinsinc_euler_angles(
                    cmd_req.goal_position
                )
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
                            values=[cmd_req.speed_limit.motor_1],
                        ),
                        InterfaceValue(
                            interface_names=["speed_limit"],
                            values=[cmd_req.speed_limit.motor_2],
                        ),
                        InterfaceValue(
                            interface_names=["speed_limit"],
                            values=[cmd_req.speed_limit.motor_3],
                        ),
                    ]
                )
            if cmd_req.HasField("torque_limit"):
                raw_commands.extend(
                    [
                        InterfaceValue(
                            interface_names=["torque_limit"],
                            values=[cmd_req.torque_limit.motor_1],
                        ),
                        InterfaceValue(
                            interface_names=["torque_limit"],
                            values=[cmd_req.torque_limit.motor_2],
                        ),
                        InterfaceValue(
                            interface_names=["torque_limit"],
                            values=[cmd_req.torque_limit.motor_3],
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

        if cmd.joint_names:
            self.logger.info(f"Publishing command: {cmd}")
            self.bridge_node.publish_command(cmd)

        return Empty()

    def StreamCommand(
        self, request_stream: Iterator[Orbita3DCommand], context: grpc.ServicerContext
    ) -> Empty:
        for request in request_stream:
            self.SendCommand(request, context)
        return Empty()

    def Audit(
        self, request: ComponentId, context: grpc.ServicerContext
    ) -> Orbita3DStatus:
        return Orbita3DStatus()

    def HeartBeat(self, request: ComponentId, context: grpc.ServicerContext) -> Empty:
        return Empty()

    def Restart(self, request: ComponentId, context: grpc.ServicerContext) -> Empty:
        return Empty()

    def get_orbita3d_components(self, component_id: ComponentId, context: grpc.ServicerContext) -> Orbita3DComponents:
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
            orbita3d_raw_motor_1 = components.get_by_name(
                f"{orbita3d.name}_raw_motor_1"
            )
            orbita3d_raw_motor_2 = components.get_by_name(
                f"{orbita3d.name}_raw_motor_2"
            )
            orbita3d_raw_motor_3 = components.get_by_name(
                f"{orbita3d.name}_raw_motor_3"
            )

            self._lazy_components[c.id] = Orbita3DComponents(
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
    "present_position": lambda o: Rotation3D(
        rpy=ExtEulerAngles(
            roll=o.roll.state["position"],
            pitch=o.pitch.state["position"],
            yaw=o.yaw.state["position"],
        ),
    ),
    "present_velocity": lambda o: Vector3D(
        x=FloatValue(value=o.roll.state["velocity"]),
        y=FloatValue(value=o.pitch.state["velocity"]),
        z=FloatValue(value=o.yaw.state["velocity"]),
    ),
    "present_load": lambda o: Vector3D(
        x=FloatValue(value=o.roll.state["effort"]),
        y=FloatValue(value=o.pitch.state["effort"]),
        z=FloatValue(value=o.yaw.state["effort"]),
    ),
    "compliant": lambda o: BoolValue(value=not o.actuator.state["torque"]),
    "goal_position": lambda o: Rotation3D(
        rpy=ExtEulerAngles(
            roll=o.roll.state["target_position"],
            pitch=o.pitch.state["target_position"],
            yaw=o.yaw.state["target_position"],
        ),
    ),
    "speed_limit": lambda o: Float3D(
        motor_1=FloatValue(value=o.raw_motor_1.state["speed_limit"]),
        motor_2=FloatValue(value=o.raw_motor_2.state["speed_limit"]),
        motor_3=FloatValue(value=o.raw_motor_3.state["speed_limit"]),
    ),
    "torque_limit": lambda o: Float3D(
        motor_1=FloatValue(value=o.raw_motor_1.state["torque_limit"]),
        motor_2=FloatValue(value=o.raw_motor_2.state["torque_limit"]),
        motor_3=FloatValue(value=o.raw_motor_3.state["torque_limit"]),
    ),
    "pid": lambda o: PID3D(
        motor_1=PIDGains(
            p=FloatValue(value=o.raw_motor_1.state["p_gain"]),
            i=FloatValue(value=o.raw_motor_1.state["i_gain"]),
            d=FloatValue(value=o.raw_motor_1.state["d_gain"]),
        ),
        motor_2=PIDGains(
            p=FloatValue(value=o.raw_motor_2.state["p_gain"]),
            i=FloatValue(value=o.raw_motor_2.state["i_gain"]),
            d=FloatValue(value=o.raw_motor_2.state["d_gain"]),
        ),
        motor_3=PIDGains(
            p=FloatValue(value=o.raw_motor_3.state["p_gain"]),
            i=FloatValue(value=o.raw_motor_3.state["i_gain"]),
            d=FloatValue(value=o.raw_motor_3.state["d_gain"]),
        ),
    ),
}
