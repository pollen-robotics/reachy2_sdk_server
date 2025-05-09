import math
from typing import Iterator

import grpc
import numpy as np
import rclpy
import reachy2_monitoring as rm
from control_msgs.msg import DynamicJointState, InterfaceValue
from google.protobuf.empty_pb2 import Empty
from google.protobuf.wrappers_pb2 import BoolValue, FloatValue
from reachy2_sdk_api.component_pb2 import ComponentId, JointLimits, PIDGains
from reachy2_sdk_api.dynamixel_motor_pb2 import (
    DynamixelMotor,
    DynamixelMotorCommand,
    DynamixelMotorField,
    DynamixelMotorGoal,
    DynamixelMotorsCommand,
    DynamixelMotorState,
    DynamixelMotorStateRequest,
    DynamixelMotorStatus,
    DynamixelMotorStreamStateRequest,
    ListOfDynamixelMotor,
)
from reachy2_sdk_api.dynamixel_motor_pb2_grpc import add_DynamixelMotorServiceServicer_to_server
from sensor_msgs.msg import JointState

from ..abstract_bridge_node import AbstractBridgeNode
from ..components import Component
from ..utils import endless_get_stream, extract_fields, get_current_timestamp


class DynamixelMotorServicer:
    default_fields = [
        DynamixelMotorField.PRESENT_POSITION,
        DynamixelMotorField.GOAL_POSITION,
        DynamixelMotorField.TEMPERATURE,
        # DynamixelMotorField.TORQUE_LIMIT,
        # DynamixelMotorField.SPEED_LIMIT,
        DynamixelMotorField.COMPLIANT,
    ]

    def __init__(
        self,
        bridge_node: AbstractBridgeNode,
        logger: rclpy.impl.rcutils_logger.RcutilsLogger,
    ) -> None:
        self.bridge_node = bridge_node
        self.logger = logger

    def register_to_server(self, server: grpc.Server):
        self.logger.info("Registering 'DynamixelMotorServiceServicer' to server.")
        add_DynamixelMotorServiceServicer_to_server(self, server)

    @classmethod
    def get_info(cls, dynamixel_motor: Component) -> DynamixelMotor:
        return DynamixelMotor(
            id=ComponentId(
                id=dynamixel_motor.id,
                name=dynamixel_motor.name,
            ),
        )

    def GetAllDynamixelMotor(self, request: Empty, context: grpc.ServicerContext) -> ListOfDynamixelMotor:
        return ListOfDynamixelMotor(info=[self.get_info(o) for o in self.bridge_node.components.get_by_type("dynamixel")])

    # State
    def GetState(self, request: DynamixelMotorStateRequest, context: grpc.ServicerContext) -> DynamixelMotorState:
        components = self.bridge_node.components
        dxl_motor = components.get_by_component_id(request.id)
        state = extract_fields(DynamixelMotorField, request.fields, conversion_table, dxl_motor)

        state["timestamp"] = get_current_timestamp(self.bridge_node)
        state["temperature"] = FloatValue(value=40.0)
        state["joint_limits"] = JointLimits(min=FloatValue(value=0.0), max=FloatValue(value=100.0))
        return DynamixelMotorState(**state)

    def StreamState(
        self, request: DynamixelMotorStreamStateRequest, context: grpc.ServicerContext
    ) -> Iterator[DynamixelMotorState]:
        return endless_get_stream(
            self.GetState,
            request.req,
            context,
            1,
            # 1 / request.freq,
        )

    def build_command(self, request: DynamixelMotorsCommand, context: grpc.ServicerContext) -> DynamicJointState:
        cmd = DynamicJointState()
        cmd.joint_names = []

        for req_cmd in request.cmd:
            if not req_cmd.HasField("id"):
                context.abort(grpc.StatusCode.INVALID_ARGUMENT, "Missing 'id' field.")

            components = self.bridge_node.components

            dxl_motor = components.get_by_component_id(req_cmd.id)
            if dxl_motor is None:
                dxl_motor = components.get_by_component_name(req_cmd.name)

            if dxl_motor is None:
                context.abort(grpc.StatusCode.NOT_FOUND, "Component not found.")
            else:
                if req_cmd.HasField("compliant"):
                    cmd.joint_names.append(dxl_motor.name)
                    cmd.interface_values.append(
                        InterfaceValue(
                            interface_names=["torque"],
                            values=[not req_cmd.compliant.value],
                        )
                    )

                if req_cmd.HasField("goal_position"):
                    cmd.joint_names.append(dxl_motor.name)
                    cmd.interface_values.append(
                        InterfaceValue(
                            interface_names=["position"],
                            values=[req_cmd.goal_position.value],
                        )
                    )

                if req_cmd.HasField("speed_limit"):
                    cmd.joint_names.append(dxl_motor.name)
                    cmd.interface_values.append(
                        InterfaceValue(
                            interface_names=["speed_limit"],
                            values=[req_cmd.speed_limit.value],
                        )
                    )

                if req_cmd.HasField("torque_limit"):
                    cmd.joint_names.append(dxl_motor.name)
                    cmd.interface_values.append(
                        InterfaceValue(
                            interface_names=["torque_limit"],
                            values=[req_cmd.torque_limit.value],
                        )
                    )
        return cmd

    # Command
    def SendCommand(self, request: DynamixelMotorsCommand, context: grpc.ServicerContext) -> Empty:
        cmd = self.build_command(request, context)

        if cmd.joint_names:
            self.bridge_node.publish_command(cmd)

        return Empty()

    def SetPosition(self, request: DynamixelMotorGoal, context: grpc.ServicerContext) -> Empty:
        return Empty()

    def StreamCommand(self, request_stream: Iterator[DynamixelMotorsCommand], context: grpc.ServicerContext) -> Empty:
        for request in request_stream:
            self.SendCommand(request, context)
        return Empty()

    # Doctor
    def Audit(self, request: ComponentId, context: grpc.ServicerContext) -> DynamixelMotorStatus:
        return DynamixelMotorStatus()

    def HeartBeat(self, request: ComponentId, context: grpc.ServicerContext) -> Empty:
        return Empty()

    def Restart(self, request: ComponentId, context: grpc.ServicerContext) -> Empty:
        return Empty()


conversion_table = {
    "id": lambda o: ComponentId(id=o.id, name=o.name),
    "present_position": lambda o: FloatValue(value=o.state["position"]),
    "present_speed": lambda o: FloatValue(value=o.state["velocity"]),
    "present_load": lambda o: FloatValue(value=o.state["effort"]),
    "compliant": lambda o: BoolValue(value=not o.state["torque"]),
    "goal_position": lambda o: FloatValue(value=o.state["target_position"]),
    "speed_limit": lambda o: FloatValue(value=o.state["speed_limit"]),
    "torque_limit": lambda o: FloatValue(value=o.state["torque_limit"]),
    "pid": lambda o: PIDGains(
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
}
