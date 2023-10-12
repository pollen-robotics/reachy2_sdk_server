import grpc
import rclpy

from control_msgs.msg import DynamicJointState, InterfaceValue


from google.protobuf.empty_pb2 import Empty

from reachy_sdk_api_v2.arm_pb2 import (
    Arm,
    ArmCartesianGoal,
    ArmDescription,
    ArmFKRequest,
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
from .orbita2d import Orbita2dServicer
from .orbita3d import Orbita3dServicer


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
        arms = self.bridge_node.parts.get_by_type("arm")

        return ListOfArm(
            arm=[
                Arm(
                    part_id=PartId(name=arm.name, id=arm.id),
                    description=ArmDescription(
                        shoulder=Orbita2dServicer.get_info(
                            self.bridge_node.components.get_by_name(
                                arm.components[0].name
                            )
                        ),
                        elbow=Orbita2dServicer.get_info(
                            self.bridge_node.components.get_by_name(
                                arm.components[1].name
                            )
                        ),
                        wrist=Orbita3dServicer.get_info(
                            self.bridge_node.components.get_by_name(
                                arm.components[2].name
                            )
                        ),
                    ),
                )
                for arm in arms
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
    def set_stiffness(self, request: PartId, torque: bool) -> None:
        part = self.bridge_node.parts.get_by_part_id(request)

        cmd = DynamicJointState()
        cmd.joint_names = []

        for c in part.components:
            cmd.joint_names.append(c.name)
            cmd.interface_values.append(
                InterfaceValue(
                    interface_names=["torque"],
                    values=[torque],
                )
            )

        self.logger.info(f"Publishing command: {cmd}")
        self.bridge_node.publish_command(cmd)

    def TurnOn(self, request: PartId, context: grpc.ServicerContext) -> Empty:
        self.set_stiffness(request, torque=True)
        return Empty()

    def TurnOff(self, request: PartId, context: grpc.ServicerContext) -> Empty:
        self.set_stiffness(request, torque=False)
        return Empty()

    # Temperatures
    def GetTemperatures(
        self, request: PartId, context: grpc.ServicerContext
    ) -> ArmTemperatures:
        return ArmTemperatures()

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
        self, request: ArmFKRequest, context: grpc.ServicerContext
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
