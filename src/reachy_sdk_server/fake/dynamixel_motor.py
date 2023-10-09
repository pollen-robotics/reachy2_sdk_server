from typing import Dict, Iterator
import uuid
from google.protobuf.empty_pb2 import Empty
from google.protobuf.wrappers_pb2 import BoolValue, FloatValue
from google.protobuf.timestamp_pb2 import Timestamp
from grpc import ServicerContext
from reachy_sdk_api_v2.dynamixel_motor_pb2 import (
    ListOfDynamixelMotorInfo,
    DynamixelMotorCommand,
    DynamixelMotorField,
    DynamixelMotorInfo,
    DynamixelMotorState,
    DynamixelMotorStateRequest,
    DynamixelMotorStreamStateRequest,
)

from reachy_sdk_api_v2.component_pb2 import ComponentId, PIDGains
from reachy_sdk_api_v2.dynamixel_motor_pb2_grpc import DynamixelMotorServiceServicer

from .utils import endless_get_stream


class DynamixelMotorServicer(DynamixelMotorServiceServicer):
    def __init__(self) -> None:
        self.antennas: Dict[str, FakeDynamixelMotor] = {}
        self.add_dynamixel_motor(32, "dynamixel_motor_l_antenna")
        self.add_dynamixel_motor(31, "dynamixel_motor_r_antenna")

    def add_dynamixel_motor(self, id: int, name: str) -> None:
        self.antennas[name] = FakeDynamixelMotor(id, name)

    def check_component_id(self, id: str, context: ServicerContext = None) -> bool:
        if id not in self.antennas.keys():
            if context:
                context.abort(404, f"{id} not found")
            return False
        return True

    def GetAllDynamixelMotor(
        self, request: Empty, context: ServicerContext
    ) -> ListOfDynamixelMotorInfo:
        return ListOfDynamixelMotorInfo(
            info=[
                DynamixelMotorInfo(
                    id=ComponentId(id=antenna.id),
                    serial_number=antenna.serial_number,
                ) for antenna in self.antennas.values()]
        )

    def GetState(
        self, request: DynamixelMotorStateRequest, context: ServicerContext
    ) -> DynamixelMotorState:
        self.check_component_id(request.id.id, context)

        kwargs = {}
        antenna = self.antennas[request.id.id]

        timestamp = Timestamp()
        timestamp.GetCurrentTime()

        for field in request.fields:
            if field in (DynamixelMotorField.NAME, DynamixelMotorField.ALL):
                kwargs['name'] = antenna.id
            if field in (DynamixelMotorField.PRESENT_POSITION, DynamixelMotorField.ALL):
                kwargs['present_position'] = FloatValue(value=antenna.present_position)
            if field in (DynamixelMotorField.PRESENT_SPEED, DynamixelMotorField.ALL):
                kwargs['present_speed'] = FloatValue(value=antenna.present_speed)
            if field in (DynamixelMotorField.PRESENT_LOAD, DynamixelMotorField.ALL):
                kwargs['present_load'] = FloatValue(value=antenna.present_load)
            if field in (DynamixelMotorField.TEMPERATURE, DynamixelMotorField.ALL):
                kwargs['temperature'] = FloatValue(value=antenna.temperature)
            if field in (DynamixelMotorField.COMPLIANT, DynamixelMotorField.ALL):
                kwargs['compliant'] = BoolValue(value=antenna.compliant)
            if field in (DynamixelMotorField.GOAL_POSITION, DynamixelMotorField.ALL):
                kwargs['goal_position'] = FloatValue(value=antenna.goal_position)
            if field in (DynamixelMotorField.SPEED_LIMIT, DynamixelMotorField.ALL):
                kwargs['speed_limit'] = FloatValue(value=antenna.speed_limit)
            if field in (DynamixelMotorField.TORQUE_LIMIT, DynamixelMotorField.ALL):
                kwargs['torque_limit'] = FloatValue(value=antenna.torque_limit)
            if field in (DynamixelMotorField.PID, DynamixelMotorField.ALL):
                kwargs['pid'] = antenna.pid

        return DynamixelMotorState(
            timestamp=timestamp,
            **kwargs,
        )

    def StreamState(
        self, request: DynamixelMotorStreamStateRequest, context: ServicerContext
    ):
        self.check_component_id(request.req.id.id, context)
        return endless_get_stream(
            self.GetState, request.req, context, period=1 / request.freq
        )

    def SendCommand(self, request: DynamixelMotorCommand, context: ServicerContext) -> Empty:
        self.check_component_id(request.id.id, context)
        antenna = self.antennas[request.id.id]
        antenna.handle_command(request)
        return Empty()

    def StreamCommand(
        self, request_iterator: Iterator[DynamixelMotorCommand], context: ServicerContext
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

class FakeDynamixelMotor:
    def __init__(self, id: int, name: str) -> None:
        self.id = id
        self.name = name
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
        self.serial_number = uuid.uuid4().hex
        self.compliant = False

    def handle_command(self, request: DynamixelMotorCommand) -> None:
        if request.HasField('compliant'):
            self.compliant = request.compliant.value
        if request.HasField('goal_position'):
            self.goal_position = request.goal_position.value
        if request.HasField('speed_limit'):
            self.speed_limit = request.speed_limit.value
        if request.HasField('torque_limit'):
            self.torque_limit = request.torque_limit.value
