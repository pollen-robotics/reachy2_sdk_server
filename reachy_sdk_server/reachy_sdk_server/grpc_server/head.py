import grpc
import rclpy

from google.protobuf.empty_pb2 import Empty

from reachy_sdk_api_v2.component_pb2 import (
    ComponentId,
)
from reachy_sdk_api_v2.head_pb2_grpc import (
    add_HeadServiceServicer_to_server,
)
from reachy_sdk_api_v2.head_pb2 import (
    Head,
    HeadDescription,
    HeadLookAtGoal,
    HeadStatus,
    HeadTemperatures,
    HeadState,
    ListOfHead,
    JointsLimits,
    NeckFKRequest,
    NeckFKSolution,
    NeckGoal,
    NeckIKRequest,
    NeckIKSolution,
    SpeedLimitRequest,
)
from reachy_sdk_api_v2.kinematics_pb2 import (
    Rotation3D,
    Quaternion,
)
from reachy_sdk_api_v2.part_pb2 import (
    PartId,
)

from ..abstract_bridge_node import AbstractBridgeNode
from ..conversion import (
    extract_quaternion_from_pose,
    neck_position_to_joint_state,
)
from .orbita3d import (
    Orbita3dServicer,
    Orbita3DStateRequest,
)
from ..parts import Part
from ..utils import get_current_timestamp


class HeadServicer:
    def __init__(
        self,
        bridge_node: AbstractBridgeNode,
        logger: rclpy.impl.rcutils_logger.RcutilsLogger,
        orbita3d_servicer: Orbita3dServicer,
    ) -> None:
        self.bridge_node = bridge_node
        self.logger = logger

        self.orbita3d_servicer = orbita3d_servicer

        self.heads = self.bridge_node.parts.get_by_type("head")

    def register_to_server(self, server: grpc.Server):
        self.logger.info("Registering 'HeadServiceServicer' to server.")
        add_HeadServiceServicer_to_server(self, server)

    def get_head(self, head: Part, context: grpc.ServicerContext) -> Head:
        return Head(
            part_id=PartId(name=head.name, id=head.id),
            description=HeadDescription(
                neck=Orbita3dServicer.get_info(
                    self.bridge_node.components.get_by_name(head.components[0].name)
                ),
                # l_antenna=DynamixelMotor.get_info(
                #     self.bridge_node.components.get_by_name(head.components[1].name)
                # ),
                # r_antenna=DynamixelMotor.get_info(
                #     self.bridge_node.components.get_by_name(head.components[2].name)
                # ),
            ),
        )

    def GetAllHeads(self, request: Empty, context: grpc.ServicerContext) -> ListOfHead:
        return ListOfHead(heads=[self.get_head(head, context) for head in self.heads])

    def GetState(self, request: PartId, context: grpc.ServicerContext) -> HeadState:
        head = self.bridge_node.parts.get_by_id(request.id)

        return HeadState(
            timestamp=get_current_timestamp(self.bridge_node),
            id=request,
            neck_state=self.orbita3d_servicer.GetState(
                Orbita3DStateRequest(
                    fields=self.orbita3d_servicer.default_fields,
                    id=ComponentId(id=head.components[0].id),
                ),
                context,
            ),
            # l_antenna_state=self.dynamixel_servicer.GetState(
            #     DynamixelStateRequest(
            #         fields=self.dynamixel_servicer.default_fields,
            #         id=ComponentId(id=head.components[1].id),
            #     ),
            #     context,
            # ),
            # r_antenna_state=self.dynamixel_servicer.GetState(
            #     DynamixelStateRequest(
            #         fields=self.dynamixel_servicer.default_fields,
            #         id=ComponentId(id=head.components[2].id),
            #     ),
            #     context,
            # ),
        )

    def ComputeNeckFK(
        self, request: NeckFKRequest, context: grpc.ServicerContext
    ) -> NeckFKSolution:
        head = self.bridge_node.parts.get_by_part_id(request.id)
        success, pose = self.bridge_node.compute_forward(
            request.id, neck_position_to_joint_state(request.position, head)
        )

        sol = NeckFKSolution()

        if success:
            sol.success = True
            sol.orientation.q = Quaternion(extract_quaternion_from_pose(pose))

        return sol

    # rpc ComputeNeckIK (NeckIKRequest) returns (NeckIKSolution);
    def ComputeNeckIK(
        self, request: NeckIKRequest, context: grpc.ServicerContext
    ) -> NeckIKSolution:
        pass

    # rpc GoToOrientation (NeckGoal) returns (google.protobuf.Empty);
    def GoToOrientation(
        self, request: NeckGoal, context: grpc.ServicerContext
    ) -> Empty:
        pass

    # rpc GetOrientation (reachy.part.PartId) returns (reachy.kinematics.Quaternion);
    def GetOrientation(
        self, request: PartId, context: grpc.ServicerContext
    ) -> Quaternion:
        pass

    # rpc LookAt (HeadLookAtGoal) returns (google.protobuf.Empty);
    def LookAt(self, request: HeadLookAtGoal, context: grpc.ServicerContext) -> Empty:
        pass

    # rpc Audit (reachy.part.PartId) returns (HeadStatus);
    def Audit(self, request: PartId, context: grpc.ServicerContext) -> HeadStatus:
        pass

    # rpc HeartBeat (reachy.part.PartId) returns (google.protobuf.Empty);
    def HeartBeat(self, request: PartId, context: grpc.ServicerContext) -> Empty:
        pass

    # rpc Restart (reachy.part.PartId) returns (google.protobuf.Empty);
    def Restart(self, request: PartId, context: grpc.ServicerContext) -> Empty:
        pass

    # rpc ResetDefaultValues(reachy.part.PartId) returns (google.protobuf.Empty);
    def ResetDefaultValues(
        self, request: PartId, context: grpc.ServicerContext
    ) -> Empty:
        pass

    # rpc TurnOn (reachy.part.PartId) returns (google.protobuf.Empty);
    def TurnOn(self, request: PartId, context: grpc.ServicerContext) -> Empty:
        pass

    # rpc TurnOff (reachy.part.PartId) returns (google.protobuf.Empty);
    def TurnOff(self, request: PartId, context: grpc.ServicerContext) -> Empty:
        pass

    # rpc GetJointsLimits (reachy.part.PartId) returns (JointsLimits);
    def GetJointsLimits(
        self, request: PartId, context: grpc.ServicerContext
    ) -> JointsLimits:
        pass

    # rpc GetTemperatures (reachy.part.PartId) returns (HeadTemperatures);
    def GetTemperatures(
        self, request: PartId, context: grpc.ServicerContext
    ) -> HeadTemperatures:
        pass

    # rpc GetJointGoalPosition (reachy.part.PartId) returns (kinematics.Rotation3D);
    def GetJointGoalPosition(
        self, request: PartId, context: grpc.ServicerContext
    ) -> Rotation3D:
        pass

    # rpc SetSpeedLimit (SpeedLimitRequest) returns (google.protobuf.Empty);
    def SetSpeedLimit(
        self, request: SpeedLimitRequest, context: grpc.ServicerContext
    ) -> Empty:
        pass
