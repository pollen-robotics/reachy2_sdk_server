from pathlib import Path
from typing import Dict, Iterator, List
import uuid

from google.protobuf.empty_pb2 import Empty
from google.protobuf.timestamp_pb2 import Timestamp
from grpc import ServicerContext
from reachy_sdk_api_v2.arm_pb2 import (
    ArmField,
    ArmState,
    Arm,
    ListOfArm,
    ArmPosition,
    ArmCartesianGoal,
    ArmJointGoal,
    ArmEndEffector,
    ArmFKSolution,
    ArmIKSolution,
    SpeedLimit,
    SpeedLimitRequest,
    ArmTemperatures,
)
from reachy_sdk_api_v2.arm_pb2_grpc import ArmServiceServicer
from reachy_sdk_api_v2.part_pb2 import PartInfo, PartId
from reachy_sdk_api_v2.orbita2d_pb2 import (
    Orbita2DInfo,
)
from reachy_sdk_api_v2.orbita3d_pb2 import (
    Orbita3DInfo,
)
from .orbita2d import FakeOrbita2D
from .orbita3d import FakeOrbita3D
from .utils import read_config_file


class ArmServicer(ArmServiceServicer):
    def __init__(self, arms) -> None:
        self.arms = arms

    def check_component_id(self, id: str, context: ServicerContext = None) -> bool:
        if id not in self.arms.keys():
            if context:
                context.abort(404, f"{id} not found")
            return False
        return True

    def GetAllArms(self, request, context):
        return ListOfArm(
            arm=[
                Arm(
                    part_id=arm.get_part_id_msg(),
                    info=arm.get_part_info_msg(),
                ) for arm in self.arms.values()]
        )

    def ComputeArmFK(self, request, context):
        return super().ComputeArmFK(request, context)

    def ComputeArmIK(self, request, context):
        return super().ComputeArmIK(request, context)

    def GoToCartesianPosition(self, request, context):
        return super().GoToCartesianPosition(request, context)

    def GoToJointPosition(self, request: ArmJointGoal, context) -> Empty:
        self.check_component_id(request.id.name, context)
        arm = self.arms[request.id.name]
        arm.shoulder.pitch.goal_position = request.position.shoulder_pitch
        arm.shoulder.roll.goal_position = request.position.shoulder_roll
        arm.elbow.yaw.goal_position = request.position.elbow_yaw
        arm.elbow.pitch.goal_position = request.position.elbow_pitch
        arm.wrist.roll.goal_position = request.position.wrist_roll
        arm.wrist.pitch.goal_position = request.position.wrist_pitch
        arm.wrist.yaw.goal_position = request.position.wrist_yaw
        return Empty()

    def GetCartesianPosition(self, request, context):
        return super().GetCartesianPosition(request, context)

    def GetJointPosition(self, request: PartId, context) -> ArmPosition:
        self.check_component_id(request.name, context)
        arm = self.arms[request.name]
        return ArmPosition(
            shoulder_pitch=arm.shoulder.pitch.present_position,
            shoulder_roll=arm.shoulder.roll.present_position,
            elbow_yaw=arm.elbow.yaw.present_position,
            elbow_pitch=arm.elbow.pitch.present_position,
            wrist_roll=arm.wrist.roll.present_position,
            wrist_pitch=arm.wrist.pitch.present_position,
            wrist_yaw=arm.wrist.yaw.present_position,
        )

    def Audit(self, request, context):
        return super().Audit(request, context)

    def HeartBeat(self, request, context):
        return super().HeartBeat(request, context)

    def Restart(self, request, context):
        return super().Restart(request, context)

    def ResetDefaultValues(self, request, context):
        return super().ResetDefaultValues(request, context)

    def TurnOn(self, request: PartId, context) -> Empty:
        self.check_component_id(request.name, context)
        arm = self.arms[request.name]
        for act in [arm.shoulder, arm.elbow, arm.wrist]:
            act.compliant = False
        return Empty()

    def TurnOff(self, request: PartId, context) -> Empty:
        self.check_component_id(request.name, context)
        arm = self.arms[request.name]
        for act in [arm.shoulder, arm.elbow, arm.wrist]:
            act.compliant = True
        return Empty()

    def GetJointsLimits(self, request, context):
        return super().GetJointsLimits(request, context)

    def GetTemperatures(self, request, context):
        return super().GetTemperatures(request, context)

    def GetJointGoalPosition(self, request, context):
        return super().GetJointGoalPosition(request, context)

    def SetSpeedLimit(self, request, context):
        return super().SetSpeedLimit(request, context)


class FakeArm:
    def __init__(
            self,
            side: str,
            orbita2ds: List[Dict[str, FakeOrbita2D]],
            orbita3ds: List[Dict[str, FakeOrbita3D]]) -> None:
        self.side = side
        self.name = f'{side}_arm'
        self.serial_number = uuid.uuid4().hex
        self.version_hard = '1.0.0'
        self.version_soft = '1.8.1'
        self.add_actuators(orbita2ds, orbita3ds)

    def get_part_id_msg(self) -> PartId:
        return PartId(
            name=self.name,
        )

    def get_part_info_msg(self) -> PartInfo:
        return PartInfo(
            serial_number=self.serial_number,
            version_hard=self.version_hard,
            version_soft=self.version_soft,
        )

    def add_actuators(
            self,
            orbita2ds: List[Dict[str, FakeOrbita2D]],
            orbita3ds: List[Dict[str, FakeOrbita3D]]) -> None:
        config = read_config_file(
            f'{Path.cwd()}/config/{self.side}_arm.yaml'
        )[f'{self.side}_arm']
        for sub_part in config:
            if config[sub_part]['actuator'] == 'orbita2d':
                setattr(self, sub_part, orbita2ds[int(config[sub_part]["id"])])
            elif config[sub_part]['actuator'] == 'orbita3d':
                setattr(self, sub_part, orbita3ds[int(config[sub_part]["id"])])
