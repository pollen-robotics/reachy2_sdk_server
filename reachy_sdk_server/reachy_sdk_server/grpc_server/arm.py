import grpc
import rclpy

from google.protobuf.empty_pb2 import Empty

from reachy_sdk_api_v2.arm_pb2 import (
    Arm,
    ArmCartesianGoal,
    ArmDescription,
    ArmFKSolution,
    ArmIKRequest,
    ArmIKSolution,
    ArmJointGoal,
    ArmPosition,
    ArmStatus,
    ArmTemperatures,
    ArmLimits,
    ListOfArm,
    SpeedLimitRequest,
)
from reachy_sdk_api_v2.arm_pb2_grpc import (
    add_ArmServiceServicer_to_server,
)
from reachy_sdk_api_v2.part_pb2 import PartId, PartInfo
from reachy_sdk_api_v2.kinematics_pb2 import Matrix4x4


from ..abstract_bridge_node import AbstractBridgeNode


class ArmServicer:
    def __init__(
        self,
        bridge_node: AbstractBridgeNode,
        logger: rclpy.impl.rcutils_logger.RcutilsLogger,
    ) -> None:
        self.bridge_node = bridge_node
        self.logger = logger

    def register_to_server(self, server: grpc.Server):
        self.logger.info("Registering 'ArmServiceServicer' to server.")
        add_ArmServiceServicer_to_server(self, server)

    def GetAllArms(self, request: Empty, context: grpc.ServicerContext) -> ListOfArm:
        return ListOfArm(
            [
                Arm(
                    part_id=PartId(name="", id=""),
                    info=PartInfo(
                        serial_number="",
                        versoin_hard="",
                        version_soft="",
                    ),
                    description=ArmDescription(),
                )
            ]
        )

    # Position and GoTo
    def GoToCartesianPosition(
        self, request: ArmCartesianGoal, context: grpc.ServicerContext
    ) -> Empty:
        return Empty()

    def GoToJointPosition(
        self, request: ArmJointGoal, context: grpc.ServicerContext
    ) -> Empty:
        return Empty()

    def GetCartesianPosition(
        self, request: PartId, context: grpc.ServicerContext
    ) -> Matrix4x4:
        return Matrix4x4()

    def GetJointPosition(
        self, request: PartId, context: grpc.ServicerContext
    ) -> ArmPosition:
        return ArmPosition()

    def GetJointGoalPosition(
        self, request: PartId, context: grpc.ServicerContext
    ) -> ArmPosition:
        return ArmPosition()

    # Compliances
    def TurnOn(self, request: PartId, context: grpc.ServicerContext) -> Empty:
        return Empty()

    def TurnOff(self, request: PartId, context: grpc.ServicerContext) -> Empty:
        return Empty()

    # Temperatures
    def GetTemperatures(
        self, request: PartId, context: grpc.ServicerContext
    ) -> ArmTemperatures:
        return 0.0

    # Position and Speed limit
    def GetJointsLimits(
        self, request: PartId, context: grpc.ServicerContext
    ) -> ArmLimits:
        return ArmLimits()

    def SetSpeedLimit(
        self, request: SpeedLimitRequest, context: grpc.ServicerContext
    ) -> Empty:
        return Empty()

    # Kinematics
    def ComputeArmFK(
        self, request: ArmPosition, context: grpc.ServicerContext
    ) -> ArmFKSolution:
        return ArmFKSolution()

    def ComputeArmIK(
        self, request: ArmIKRequest, context: grpc.ServicerContext
    ) -> ArmIKSolution:
        return ArmIKSolution()

    # Doctor
    def Audit(self, request: PartId, context: grpc.ServicerContext) -> ArmStatus:
        return ArmStatus()

    def HeartBeat(self, request: PartId, context: grpc.ServicerContext) -> Empty:
        return Empty()

    def Restart(self, request: PartId, context: grpc.ServicerContext) -> Empty:
        return Empty()

    def ResetDefaultValues(
        self, request: PartId, context: grpc.ServicerContext
    ) -> Empty:
        return Empty()
