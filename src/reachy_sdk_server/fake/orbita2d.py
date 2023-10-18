from typing import Dict, Iterator
import uuid
from google.protobuf.empty_pb2 import Empty
from google.protobuf.wrappers_pb2 import BoolValue
from google.protobuf.timestamp_pb2 import Timestamp
from grpc import ServicerContext
from reachy_sdk_api_v2.orbita2d_pb2 import (
    Axis,
    Float2D,
    ListOfOrbita2D,
    Orbita2DCommand,
    Orbita2DsCommand,
    Orbita2DField,
    Orbita2D,
    Orbita2DState,
    Orbita2DStateRequest,
    Orbita2DStreamStateRequest,
    PID2D,
)

from reachy_sdk_api_v2.component_pb2 import ComponentId, PIDGains
from reachy_sdk_api_v2.orbita2d_pb2_grpc import Orbita2DServiceServicer

from .utils import endless_get_stream


class Orbita2DServicer(Orbita2DServiceServicer):
    def __init__(self) -> None:
        self.orbitas: Dict[int, FakeOrbita2D] = {}
        self.add_orbita2d(10, "orbita2d_r_shoulder", "pitch",  "roll")
        self.add_orbita2d(11, "orbita2d_r_elbow", "yaw",  "pitch")
        self.add_orbita2d(20, "orbita2d_l_shoulder", "pitch",  "roll")
        self.add_orbita2d(21, "orbita2d_l_elbow", "yaw",  "pitch")

    def add_orbita2d(self, id: int, name: str, axis1_type: str, axis2_type: str) -> None:
        self.orbitas[id] = FakeOrbita2D(id, name, axis1_type, axis2_type)

    def check_component_id(self, id: int, context: ServicerContext = None) -> bool:
        if id not in self.orbitas.keys():
            if context:
                context.abort(404, f"{id} not found")
            return False
        return True

    def GetAllOrbita2D(
        self, request: Empty, context: ServicerContext
    ) -> ListOfOrbita2D:
        return ListOfOrbita2D(
            info=[
                Orbita2D(
                    id=ComponentId(id=orbita.id),
                    serial_number=orbita.serial_number,
                    axis_1=getattr(Axis, orbita._axis1_type.upper()),
                    axis_2=getattr(Axis, orbita._axis2_type.upper()),
                ) for orbita in self.orbitas.values()]
        )

    def GetState(
        self, request: Orbita2DStateRequest, context: ServicerContext
    ) -> Orbita2DState:
        self.check_component_id(request.id.id, context)

        kwargs = {}
        orbita = self.orbitas[request.id.id]

        timestamp = Timestamp()
        timestamp.GetCurrentTime()

        for field in request.fields:
            if field in (Orbita2DField.NAME, Orbita2DField.ALL):
                kwargs['name'] = orbita.id
            if field in (Orbita2DField.PRESENT_POSITION, Orbita2DField.ALL):
                kwargs['present_position'] = orbita.get_float2d_message('present_position')
            if field in (Orbita2DField.PRESENT_SPEED, Orbita2DField.ALL):
                kwargs['present_speed'] = orbita.get_float2d_message('present_speed')
            if field in (Orbita2DField.PRESENT_LOAD, Orbita2DField.ALL):
                kwargs['present_load'] = orbita.get_float2d_message('present_load')
            if field in (Orbita2DField.TEMPERATURE, Orbita2DField.ALL):
                kwargs['temperature'] = orbita.get_float2d_message('temperature')
            if field in (Orbita2DField.COMPLIANT, Orbita2DField.ALL):
                kwargs['compliant'] = BoolValue(value=orbita.compliant)
            if field in (Orbita2DField.GOAL_POSITION, Orbita2DField.ALL):
                kwargs['goal_position'] = orbita.get_float2d_message('goal_position')
            if field in (Orbita2DField.SPEED_LIMIT, Orbita2DField.ALL):
                kwargs['speed_limit'] = orbita.get_float2d_message('speed_limit')
            if field in (Orbita2DField.TORQUE_LIMIT, Orbita2DField.ALL):
                kwargs['torque_limit'] = orbita.get_float2d_message('torque_limit')
            if field in (Orbita2DField.PID, Orbita2DField.ALL):
                kwargs['pid'] = orbita.get_pid2d_message()

        return Orbita2DState(
            timestamp=timestamp,
            **kwargs,
        )

    def StreamState(
        self, request: Orbita2DStreamStateRequest, context: ServicerContext
    ):
        self.check_component_id(request.req.id.id, context)
        return endless_get_stream(
            self.GetState, request.req, context, period=1 / request.freq
        )

    def SendCommand(self, request: Orbita2DsCommand, context: ServicerContext) -> Empty:
        for cmd in request.cmd:
            self.check_component_id(cmd.id.id, context)
            orbita = self.orbitas[cmd.id.id]
            orbita.handle_command(cmd)
        return Empty()

    def StreamCommand(
        self, request_iterator: Iterator[Orbita2DsCommand], context: ServicerContext
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


class FakeOrbita2D:
    def __init__(self, id: int, name: str, axis1_type: str, axis2_type: str) -> None:
        self.id = id
        self.name = name
        self._axis1_type = axis1_type
        self._axis2_type = axis2_type

        setattr(self, axis1_type, FakeAxis(axis1_type))
        setattr(self, axis2_type, FakeAxis(axis2_type))
        self.serial_number = uuid.uuid4().hex
        self.compliant = False

    def get_float2d_message(self, field: str) -> Float2D:
        axis_1 = getattr(self, self._axis1_type)
        axis_2 = getattr(self, self._axis2_type)

        return Float2D(
            axis_1=getattr(axis_1, field),
            axis_2=getattr(axis_2, field),
        )

    def get_pid2d_message(self) -> PID2D:
        axis_1 = getattr(self, self._axis1_type)
        axis_2 = getattr(self, self._axis2_type)

        return PID2D(
            gains_axis_1=axis_1.pid,
            gains_axis_2=axis_2.pid,
        )

    def handle_command(self, request: Orbita2DCommand) -> None:
        axis_1 = getattr(self, self._axis1_type)
        axis_2 = getattr(self, self._axis2_type)

        if request.HasField('compliant'):
            self.compliant = request.compliant.value
        if request.HasField('goal_position'):
            gp = request.goal_position
            if gp.axis_1 is not None:
                axis_1.goal_position = gp.axis_1
            if gp.axis_2 is not None:
                axis_2.goal_position = gp.axis_2
        if request.HasField('speed_limit'):
            sl = request.speed_limit
            if sl.motor_1 is not None:
                axis_1.speed_limit = sl.motor_1
            if sl.motor_2 is not None:
                axis_2.speed_limit = sl.motor_2
        if request.HasField('torque_limit'):
            tl = request.torque_limit
            if tl.motor_1 is not None:
                axis_1.torque_limit = tl.motor_1
            if tl.motor_2 is not None:
                axis_2.torque_limit = tl.motor_2
