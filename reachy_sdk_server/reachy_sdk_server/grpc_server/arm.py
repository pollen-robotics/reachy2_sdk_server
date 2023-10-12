import grpc
import rclpy

from control_msgs.msg import DynamicJointState, InterfaceValue
from pollen_msgs.srv import GetForwardKinematics, GetInverseKinematics

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
from ..conversion import (
    pose_to_matrix,
    rotation3d_as_extrinsinc_euler_angles,
)
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

        self.arms = self.bridge_node.parts.get_by_type("arm")

        # Register to forward/inverse kinematics ROS services
        self.forward_kinematics_clients = {}
        self.inverse_kinematics_clients = {}

        for arm in self.arms:
            c = self.bridge_node.create_client(
                srv_type=GetForwardKinematics,
                srv_name=f"/{arm.name}/forward_kinematics",
            )
            self.logger.info(f"Subscribing for service '{c.srv_name}'...")
            c.wait_for_service()
            self.forward_kinematics_clients[arm.id] = c

            c = self.bridge_node.create_client(
                srv_type=GetInverseKinematics,
                srv_name=f"/{arm.name}/inverse_kinematics",
            )
            self.logger.info(f"Subscribing for service '{c.srv_name}'...")
            c.wait_for_service()
            self.inverse_kinematics_clients[arm.id] = c

    def register_to_server(self, server: grpc.Server):
        self.logger.info("Registering 'ArmServiceServicer' to server.")
        add_ArmServiceServicer_to_server(self, server)

    def GetAllArms(self, request: Empty, context: grpc.ServicerContext) -> ListOfArm:
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
                for arm in self.arms
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
        arm = self.bridge_node.parts.get_by_part_id(request.id)

        shoulder_pos = request.position.shoulder_position
        elbow_pos = request.position.elbow_position
        wrist_pos = rotation3d_as_extrinsinc_euler_angles(
            request.position.wrist_position
        )

        joint_name = []
        for c in arm.components:
            joint_name.extend(c.get_all_joints())

        joint_pos = [
            shoulder_pos.axis_1,
            shoulder_pos.axis_2,
            elbow_pos.axis_1,
            elbow_pos.axis_2,
            wrist_pos[0],
            wrist_pos[1],
            wrist_pos[2],
        ]

        client = self.get_service(request.id, forward=True)
        req = GetForwardKinematics.Request()
        req.joint_position.name = joint_name
        req.joint_position.position = joint_pos

        resp = client.call(req)

        sol = ArmFKSolution()
        sol.success = resp.success

        if resp.success:
            sol.end_effector.pose.data.extend(pose_to_matrix(resp.pose))
        return sol

    def ComputeArmIK(
        self, request: ArmIKRequest, context: grpc.ServicerContext
    ) -> ArmIKSolution:
        return ArmIKSolution()

    def get_service(self, part_id: PartId, forward: bool):
        part = self.bridge_node.parts.get_by_part_id(part_id)

        if forward:
            return self.forward_kinematics_clients[part.id]
        else:
            return self.inverse_kinematics_clients[part.id]

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
