from typing import Dict, Iterator
import uuid
from google.protobuf.empty_pb2 import Empty
from google.protobuf.wrappers_pb2 import BoolValue
from google.protobuf.timestamp_pb2 import Timestamp
from grpc import ServicerContext
from reachy_sdk_api_v2.orbita3d_pb2 import (
    Float3D,
    ListOfOrbita3D,
    Orbita3DCommand,
    Orbita3DsCommand,
    Orbita3DField,
    Orbita3D,
    Orbita3DState,
    Orbita3DStateRequest,
    Orbita3DStreamStateRequest,
    PID3D,
)

from reachy_sdk_api_v2.component_pb2 import ComponentId, PIDGains
from reachy_sdk_api_v2.orbita3d_pb2_grpc import Orbita3DServiceServicer

from .utils import endless_get_stream


class Orbita3DServicer(Orbita3DServiceServicer):
    def __init__(self) -> None:
        self.orbitas: Dict[int, FakeOrbita3D] = {}
        self.add_orbita3d(12, "orbita3d_r_wrist")
        self.add_orbita3d(13, "orbita3d_l_wrist")
        self.add_orbita3d(30, "orbita3d_neck")

    def add_orbita3d(self, id: int, name: str) -> None:
        self.orbitas[id] = FakeOrbita3D(id, name)

    def check_component_id(self, id: int, context: ServicerContext = None) -> bool:
        if id not in self.orbitas.keys():
            if context:
                context.abort(404, f"{id} not found")
            return False
        return True

    def GetAllOrbita3D(
        self, request: Empty, context: ServicerContext
    ) -> ListOfOrbita3D:
        return ListOfOrbita3D(
            info=[
                Orbita3D(
                    id=ComponentId(id=orbita.id),
                    serial_number=orbita.serial_number,
                ) for orbita in self.orbitas.values()]
        )

    def GetState(
        self, request: Orbita3DStateRequest, context: ServicerContext
    ) -> Orbita3DState:
        self.check_component_id(request.id.id, context)

        kwargs = {}
        orbita = self.orbitas[request.id.id]

        timestamp = Timestamp()
        timestamp.GetCurrentTime()

        for field in request.fields:
            if field in (Orbita3DField.NAME, Orbita3DField.ALL):
                kwargs['name'] = orbita.id
            if field in (Orbita3DField.PRESENT_POSITION, Orbita3DField.ALL):
                kwargs['present_position'] = orbita.get_float3d_message('present_position')
            if field in (Orbita3DField.PRESENT_SPEED, Orbita3DField.ALL):
                kwargs['present_speed'] = orbita.get_float3d_message('present_speed')
            if field in (Orbita3DField.PRESENT_LOAD, Orbita3DField.ALL):
                kwargs['present_load'] = orbita.get_float3d_message('present_load')
            if field in (Orbita3DField.TEMPERATURE, Orbita3DField.ALL):
                kwargs['temperature'] = orbita.get_float3d_message('temperature')
            if field in (Orbita3DField.COMPLIANT, Orbita3DField.ALL):
                kwargs['compliant'] = BoolValue(value=orbita.compliant)
            if field in (Orbita3DField.GOAL_POSITION, Orbita3DField.ALL):
                kwargs['goal_position'] = orbita.get_float3d_message('goal_position')
            if field in (Orbita3DField.SPEED_LIMIT, Orbita3DField.ALL):
                kwargs['speed_limit'] = orbita.get_float3d_message('speed_limit')
            if field in (Orbita3DField.TORQUE_LIMIT, Orbita3DField.ALL):
                kwargs['torque_limit'] = orbita.get_float3d_message('torque_limit')
            if field in (Orbita3DField.PID, Orbita3DField.ALL):
                kwargs['pid'] = orbita.get_pid2d_message()

        return Orbita3DState(
            timestamp=timestamp,
            **kwargs,
        )

    def StreamState(
        self, request: Orbita3DStreamStateRequest, context: ServicerContext
    ):
        self.check_component_id(request.req.id.id, context)
        return endless_get_stream(
            self.GetState, request.req, context, period=1 / request.freq
        )

    def SendCommand(self, request: Orbita3DsCommand, context: ServicerContext) -> Empty:
        for cmd in request.cmd:
            self.check_component_id(cmd.id.id, context)
            orbita = self.orbitas[cmd.id.id]
            orbita.handle_command(cmd)
        return Empty()

    def StreamCommand(
        self, request_iterator: Iterator[Orbita3DsCommand], context: ServicerContext
    ) -> Empty:
        for request in request_iterator:
            self.SendCommand(request, context)
        return Empty()

    def Audit(self, request, context: ServicerContext) -> Empty:
        self.check_component_id(request.id.id, context)
        return Empty()

    def HeartBeat(self, request, context: ServicerContext) -> Empty:
        self.check_component_id(request.id.id, context)
        return Empty()

    def Restart(self, request, context: ServicerContext) -> Empty:
        self.check_component_id(request.id.id, context)
        return Empty()


class FakeAxis:
    def __init__(self, axis_type: str) -> None:
        self.axis_type = axis_type
        self.present_position = 10.0
        self.present_speed = 20.0
        self.present_load = 30.0
        self.temperature = 40.0
        self.goal_position = 50.0
        self.speed_limit = 60.0
        self.torque_limit = 70.0

        self.pid = PIDGains(
            p=1.0,
            i=1.0,
            d=1.0,
        )


class FakeOrbita3D:
    def __init__(self, id: int, name: str) -> None:
        self.id = id
        self.name = name
        self.roll = FakeAxis('roll')
        self.pitch = FakeAxis('pitch')
        self.yaw = FakeAxis('yaw')
        self.serial_number = uuid.uuid4().hex
        self.compliant = False

    def get_float3d_message(self, field: str) -> Float3D:
        return Float3D(
            roll=getattr(self.roll, field),
            pitch=getattr(self.pitch, field),
            yaw=getattr(self.yaw, field),
        )

    def get_pid2d_message(self) -> PID3D:
        return PID3D(
            roll=self.roll.pid,
            pitch=self.pitch.pid,
            yaw=self.yaw.pid,
        )

    def handle_command(self, request: Orbita3DCommand) -> None:
        if request.HasField('compliant'):
            self.compliant = request.compliant.value
        if request.HasField('goal_position'):
            # TODO: handle properly
            gp = request.goal_position.q
            if gp.x is not None:
                self.roll.goal_position = gp.x
            if gp.y is not None:
                self.pitch.goal_position = gp.y
            if gp.z is not None:
                self.yaw.goal_position = gp.z
        if request.HasField('speed_limit'):
            sl = request.speed_limit
            if sl.motor_1 is not None:
                self.roll.speed_limit = sl.motor_1
            if sl.motor_2 is not None:
                self.pitch.speed_limit = sl.motor_2
            if sl.motor_3 is not None:
                self.yaw.speed_limit = sl.motor_3
        if request.HasField('torque_limit'):
            tl = request.torque_limit
            if tl.motor_1 is not None:
                self.roll.torque_limit = tl.motor_1
            if tl.motor_2 is not None:
                self.pitch.torque_limit = tl.motor_2
            if tl.motor_3 is not None:
                self.yaw.torque_limit = tl.motor_3
