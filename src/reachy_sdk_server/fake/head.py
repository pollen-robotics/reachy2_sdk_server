from typing import Dict, Iterator, List
import uuid

from google.protobuf.empty_pb2 import Empty
from google.protobuf.timestamp_pb2 import Timestamp
from grpc import ServicerContext
from reachy_sdk_api_v2.head_pb2 import (
    HeadField,
    HeadState,
    Head,
    ListOfHead,
    HeadPosition,
    SpeedLimit,
    SpeedLimitRequest,
    HeadTemperatures,
    NeckFKRequest,
    NeckFKSolution,
    NeckOrientation,
    NeckIKRequest,
    NeckIKSolution,
)
from reachy_sdk_api_v2.head_pb2_grpc import HeadServiceServicer
from reachy_sdk_api_v2.part_pb2 import PartInfo, PartId
from reachy_sdk_api_v2.kinematics_pb2 import Quaternion
from reachy_sdk_api_v2.orbita3d_pb2 import (
    Orbita3DInfo,
)
from reachy_sdk_api_v2.kinematics_pb2 import (
    Rotation3D,
    ExtEulerAngles,
)
from reachy_sdk_api_v2.dynamixel_motor_pb2 import (
    DynamixelMotorInfo,
)
from .dynamixel_motor import FakeDynamixelMotor
from .orbita3d import FakeOrbita3D
from .utils import read_config_file


class HeadServicer(HeadServiceServicer):
    def __init__(self, heads) -> None:
        self.heads = heads

    def check_component_id(self, id: str, context: ServicerContext = None) -> bool:
        if id not in self.heads.keys():
            if context:
                context.abort(404, f"{id} not found")
            return False
        return True

    def GetAllHeads(self, request, context):
        return ListOfHead(
            head=[
                Head(
                    part_id=head.get_part_id_msg(),
                    info=head.get_part_info_msg(),
                ) for head in self.heads.values()]
        )

    def ComputeNeckFK(self, request: NeckFKRequest, context) -> NeckFKSolution:
        res = NeckFKSolution(
            success=True,
            orientation=NeckOrientation(q=Quaternion(w=1, x=0, y=0, z=0))
        )
        return res

    def ComputeNeckIK(self, request: NeckIKRequest, context) -> NeckIKSolution:
        res = NeckIKSolution(
            success=True,
            position=Rotation3D(rpy=ExtEulerAngles(roll=1, pitch=2, yaw=3))
        )
        return res

    def GoToOrientation(self, request, context):
        return super().GoToOrientation(request, context)

    def GetOrientation(self, request: PartId, context) -> Quaternion:
        self.check_component_id(request.name, context)
        head = self.heads[request.name]
        return Quaternion(
            x=0.0,
            y=0.0,
            z=0.0,
            w=1.0,
        )

    def LookAt(self, request, context):
        return super().LookAt(request, context)

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
        head = self.heads[request.name]
        for act in [head.neck, head.l_antenna, head.r_antenna]:
            act.compliant = False
        return Empty()

    def TurnOff(self, request: PartId, context) -> Empty:
        self.check_component_id(request.name, context)
        head = self.heads[request.name]
        for act in [head.neck, head.l_antenna, head.r_antenna]:
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


class FakeHead:
    def __init__(
            self,
            orbita3ds: List[Dict[str, FakeOrbita3D]],
            dynamixelmotors: List[Dict[str, FakeDynamixelMotor]]) -> None:
        self.name = 'head'
        self.serial_number = uuid.uuid4().hex
        self.version_hard = '1.0.0'
        self.version_soft = '1.8.1'
        self.add_actuators(orbita3ds, dynamixelmotors)

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
            orbita3ds: List[Dict[str, FakeOrbita3D]],
            dynamixelmotors: List[Dict[str, FakeDynamixelMotor]]) -> None:
        config = read_config_file(
            '/home/demo/dev/reachy_sdk_server/src/reachy_sdk_server/config/head.yaml'
        )['head']
        for sub_part in config:
            if config[sub_part]['actuator'] == 'dynamixel_motor':
                setattr(self, sub_part, dynamixelmotors[f'dynamixel_motor_{config[sub_part]["name"]}'])
            elif config[sub_part]['actuator'] == 'orbita3d':
                setattr(self, sub_part, orbita3ds[f'orbita3d_{config[sub_part]["name"]}'])
