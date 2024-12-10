from typing import Iterator
import math

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
    DynamixelMotorField,
    DynamixelMotorState,
    DynamixelMotorStateRequest,
    DynamixelMotorStreamStateRequest,
    DynamixelMotorGoal,
    DynamixelMotorCommand,
    DynamixelMotorsCommand,
    ListOfDynamixelMotor,
    DynamixelMotorStatus,
)
from reachy2_sdk_api.dynamixel_motor_pb2_grpc import add_DynamixelMotorServiceServicer_to_server
from sensor_msgs.msg import JointState

from ..abstract_bridge_node import AbstractBridgeNode
from ..components import Component
from ..utils import get_current_timestamp, endless_get_stream


class DynamixelMotorServicer:
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
        return ListOfDynamixelMotor(orbita2d=[self.get_info(o) for o in self.bridge_node.components.get_by_type("dynamixel_motor")])

    # State
    def GetState(self, request: DynamixelMotorStateRequest, context: grpc.ServicerContext) -> DynamixelMotorState:
        orbita2d_components = self.get_orbita2d_components(request.id, context=context)
        state = extract_fields(Orbita2dField, request.fields, conversion_table, orbita2d_components)

        state["timestamp"] = get_current_timestamp(self.bridge_node)
        state["temperature"] = FloatValue(value=40.0)
        state["joint_limits"] = JointLimits(min=FloatValue(value=0.0), max=FloatValue(value=100.0))
        return DynamixelMotorState(**state)

    def StreamState(self, request: DynamixelMotorStreamStateRequest, context: grpc.ServicerContext) -> Iterator[DynamixelMotorState]:
        return endless_get_stream(
            self.GetState,
            request.req,
            context,
            1,
            # 1 / request.freq,
        )

    # Command
    def SendCommand(self, request: DynamixelMotorsCommand, context: grpc.ServicerContext) -> Empty:
        cmd = DynamicJointState()
        cmd.joint_names = []

        return Empty()

    def SetPosition(self, request: DynamixelMotorGoal, context: grpc.ServicerContext) -> Empty:
        pass

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
    "id": lambda o: ComponentId(id=o.actuator.id, name=o.actuator.name),
    "present_position": lambda o: FloatValue(value=o.actuator.state["position"]),
    "present_speed": lambda o: FloatValue(value=o.actuator.state["velocity"]),
    "present_load": lambda o: FloatValue(value=o.actuator.state["effort"]),
    "compliant": lambda o: BoolValue(value=not o.actuator.state["torque"]),
    "goal_position": lambda o: FloatValue(value=o.actuator.state["target_position"]),
    "speed_limit": lambda o: FloatValue(value=o.actuator.state["speed_limit"]),
    "torque_limit": lambda o: FloatValue(value=o.actuator.state["torque_limit"]),
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
