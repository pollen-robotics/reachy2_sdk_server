import grpc
import time

from typing import Iterator

from google.protobuf.empty_pb2 import Empty
from google.protobuf.wrappers_pb2 import BoolValue, FloatValue

from reachy_sdk_api_v2.reachy_pb2_grpc import ReachyServiceServicer
from reachy_sdk_api_v2.reachy_pb2 import (
    Reachy,
    ReachyId,
    ReachyState,
    ReachyStreamStateRequest,
    ReachyInfo,
)

from reachy_sdk_api_v2.part_pb2 import PartId
from reachy_sdk_api_v2.arm_pb2 import Arm, ArmDescription, ArmState
from reachy_sdk_api_v2.head_pb2 import Head, HeadDescription, HeadState

from reachy_sdk_api_v2.component_pb2 import ComponentId, PIDGains
from reachy_sdk_api_v2.orbita2d_pb2 import (
    Orbita2D,
    Axis,
    Orbita2DState,
    Float2D,
    PID2D,
    Pose2D,
    Vector2D
)
from reachy_sdk_api_v2.orbita3d_pb2 import (
    Orbita3D,
    Orbita3DState,
    Float3D,
    PID3D,
    Vector3D,
)
from reachy_sdk_api_v2.dynamixel_motor_pb2 import DynamixelMotor, DynamixelMotorState

from reachy_sdk_api_v2.kinematics_pb2 import Rotation3D, ExtEulerAngles


class ReachyServicer(ReachyServiceServicer):
    def __init__(self, right_arm, head) -> None:
        self.right_arm = right_arm
        self.head = head
        self._temp = 0.01

    def GetReachy(self, request: Empty, context) -> Reachy:
        return Reachy(
            r_arm=Arm(
                part_id=PartId(
                    id=0,
                    name=self.right_arm.name,
                ),
                description=ArmDescription(
                    shoulder=Orbita2D(
                        id=ComponentId(
                            id=self.right_arm.shoulder.id,
                            name=self.right_arm.shoulder.name,
                            ),
                        serial_number=self.right_arm.shoulder.serial_number,
                        axis_1=getattr(Axis, self.right_arm.shoulder._axis1_type.upper()),
                        axis_2=getattr(Axis, self.right_arm.shoulder._axis2_type.upper()),
                    ),
                    elbow=Orbita2D(
                        id=ComponentId(
                            id=self.right_arm.elbow.id,
                            name=self.right_arm.elbow.name,
                            ),
                        serial_number=self.right_arm.elbow.serial_number,
                        axis_1=getattr(Axis, self.right_arm.elbow._axis1_type.upper()),
                        axis_2=getattr(Axis, self.right_arm.elbow._axis2_type.upper()),
                    ),
                    wrist=Orbita3D(
                        id=ComponentId(
                            id=self.right_arm.wrist.id,
                            name=self.right_arm.wrist.name,
                            ),
                        serial_number=self.right_arm.wrist.serial_number,
                    ),
                )
            ),
            head=Head(
                part_id=PartId(
                    id=2,
                    name=self.head.name,
                ),
                description=HeadDescription(
                    neck=Orbita3D(
                        id=ComponentId(
                            id=self.head.neck.id,
                            name=self.head.neck.name,
                            ),
                        serial_number=self.head.neck.serial_number,
                    ),
                    l_antenna=DynamixelMotor(
                        id=ComponentId(
                            id=self.head.l_antenna.id,
                            name=self.head.l_antenna.name,
                            ),
                        serial_number=self.head.l_antenna.serial_number,
                    ),
                    r_antenna=DynamixelMotor(
                        id=ComponentId(
                            id=self.head.r_antenna.id,
                            name=self.head.r_antenna.name,
                            ),
                        serial_number=self.head.r_antenna.serial_number,
                    ),
                )
            ),
            info=ReachyInfo(
                serial_number="Coucou c'est moi",
                version_hard="c'est hard",
                version_soft="c'est soft"
            ),
        )

    def GetReachyState(self, id: ReachyId, context: grpc.ServicerContext) -> ReachyState:
        """Get the requested joints id."""
        self._temp += 0.01
        return ReachyState(
            # l_arm_state=ArmState(
            #     name=self.left_arm.name

            # ),
            r_arm_state=ArmState(
                id=PartId(
                    name=self.right_arm.name,
                    id=0
                ),
                activated=True,
                shoulder_state=Orbita2DState(
                    id=ComponentId(
                        id=self.right_arm.shoulder.id,
                        name=self.right_arm.shoulder.name
                    ),
                    temperature=Float2D(
                        motor_1=self.right_arm.shoulder.pitch.temperature,
                        motor_2=self.right_arm.shoulder.roll.temperature
                    ),
                    present_position=Pose2D(
                        axis_1=self.right_arm.shoulder.pitch.present_position,
                        axis_2=self.right_arm.shoulder.roll.present_position
                    ),
                    present_speed=Vector2D(
                        x=self.right_arm.shoulder.pitch.present_speed,
                        y=self.right_arm.shoulder.roll.present_speed
                    ),
                    present_load=Vector2D(
                        x=self.right_arm.shoulder.pitch.present_load,
                        y=self.right_arm.shoulder.roll.present_load
                    ),
                    compliant=BoolValue(value=self.right_arm.shoulder.compliant),
                    goal_position=Pose2D(
                        axis_1=self.right_arm.shoulder.pitch.goal_position,
                        axis_2=self.right_arm.shoulder.roll.goal_position
                    ),
                    speed_limit=Float2D(
                        motor_1=self.right_arm.shoulder.pitch.speed_limit,
                        motor_2=self.right_arm.shoulder.roll.speed_limit
                    ),
                    torque_limit=Float2D(
                        motor_1=self.right_arm.shoulder.pitch.torque_limit,
                        motor_2=self.right_arm.shoulder.roll.torque_limit
                    ),
                    pid=PID2D(
                        motor_1=PIDGains(
                            p=self.right_arm.shoulder.pitch.pid.p,
                            i=self.right_arm.shoulder.pitch.pid.i,
                            d=self.right_arm.shoulder.pitch.pid.d
                        ),
                        motor_2=PIDGains(
                            p=self.right_arm.shoulder.roll.pid.p,
                            i=self.right_arm.shoulder.roll.pid.i,
                            d=self.right_arm.shoulder.roll.pid.d
                        )
                    ),
                ),
                elbow_state=Orbita2DState(
                    id=ComponentId(
                        id=self.right_arm.elbow.id,
                        name=self.right_arm.elbow.name
                    ),
                    temperature=Float2D(
                        motor_1=self.right_arm.elbow.yaw.temperature,
                        motor_2=self.right_arm.elbow.pitch.temperature
                    ),
                    present_position=Pose2D(
                        axis_1=self.right_arm.elbow.yaw.present_position,
                        axis_2=self.right_arm.elbow.pitch.present_position
                    ),
                    present_speed=Vector2D(
                        x=self.right_arm.elbow.yaw.present_speed,
                        y=self.right_arm.elbow.pitch.present_speed
                    ),
                    present_load=Vector2D(
                        x=self.right_arm.elbow.yaw.present_load,
                        y=self.right_arm.elbow.pitch.present_load
                    ),
                    compliant=BoolValue(value=self.right_arm.elbow.compliant),
                    goal_position=Pose2D(
                        axis_1=self.right_arm.elbow.yaw.goal_position,
                        axis_2=self.right_arm.elbow.pitch.goal_position
                    ),
                    speed_limit=Float2D(
                        motor_1=self.right_arm.elbow.yaw.speed_limit,
                        motor_2=self.right_arm.elbow.pitch.speed_limit
                    ),
                    torque_limit=Float2D(
                        motor_1=self.right_arm.elbow.yaw.torque_limit,
                        motor_2=self.right_arm.elbow.pitch.torque_limit
                    ),
                    pid=PID2D(
                        motor_1=PIDGains(
                            p=self.right_arm.elbow.yaw.pid.p,
                            i=self.right_arm.elbow.yaw.pid.i,
                            d=self.right_arm.elbow.yaw.pid.d,
                        ),
                        motor_2=PIDGains(
                            p=self.right_arm.elbow.pitch.pid.p,
                            i=self.right_arm.elbow.pitch.pid.i,
                            d=self.right_arm.elbow.pitch.pid.d,
                        )
                    ),
                ),
                wrist_state=Orbita3DState(
                    id=ComponentId(
                        id=self.right_arm.wrist.id,
                        name=self.right_arm.wrist.name
                    ),
                    temperature=Float3D(
                        motor_1=self.right_arm.wrist.roll.temperature,
                        motor_2=self.right_arm.wrist.pitch.temperature,
                        motor_3=self.right_arm.wrist.yaw.temperature,
                    ),
                    present_position=Rotation3D(
                        rpy=ExtEulerAngles(
                            roll=0.5,
                            pitch=0.6,
                            yaw=0.7,
                        )
                    ),
                    present_speed=Vector3D(
                        x=self.right_arm.wrist.roll.present_speed,
                        y=self.right_arm.wrist.pitch.present_speed,
                        z=self.right_arm.wrist.yaw.present_speed,
                    ),
                    present_load=Vector3D(
                        x=self.right_arm.wrist.roll.present_load,
                        y=self.right_arm.wrist.pitch.present_load,
                        z=self.right_arm.wrist.yaw.present_load,
                    ),
                    compliant=BoolValue(value=self.right_arm.wrist.compliant),
                    goal_position=Rotation3D(
                        rpy=ExtEulerAngles(
                            roll=0.5,
                            pitch=0.6,
                            yaw=0.7,
                        )
                    ),
                    speed_limit=Float3D(
                        motor_1=self.right_arm.wrist.roll.speed_limit,
                        motor_2=self.right_arm.wrist.pitch.speed_limit,
                        motor_3=self.right_arm.wrist.yaw.speed_limit,
                    ),
                    torque_limit=Float3D(
                        motor_1=self.right_arm.wrist.roll.torque_limit,
                        motor_2=self.right_arm.wrist.pitch.torque_limit,
                        motor_3=self.right_arm.wrist.yaw.torque_limit,
                    ),
                    pid=PID3D(
                        motor_1=PIDGains(
                            p=self.right_arm.wrist.roll.pid.p,
                            i=self.right_arm.wrist.roll.pid.i,
                            d=self.right_arm.wrist.roll.pid.d,
                        ),
                        motor_2=PIDGains(
                            p=self.right_arm.wrist.pitch.pid.p,
                            i=self.right_arm.wrist.pitch.pid.i,
                            d=self.right_arm.wrist.pitch.pid.d,
                        ),
                        motor_3=PIDGains(
                            p=self.right_arm.wrist.yaw.pid.p,
                            i=self.right_arm.wrist.yaw.pid.i,
                            d=self.right_arm.wrist.yaw.pid.d,
                        ),
                    ),
                ),
            ),
            head_state=HeadState(
                id=PartId(
                    name=self.head.name,
                    id=2
                ),
                activated=True,
                neck_state=Orbita3DState(
                    id=ComponentId(
                        id=self.head.neck.id,
                        name=self.head.neck.name
                    ),
                    temperature=Float3D(
                        motor_1=self.head.neck.roll.temperature,
                        motor_2=self.head.neck.pitch.temperature,
                        motor_3=self.head.neck.yaw.temperature,
                    ),
                    present_position=Rotation3D(
                        rpy=ExtEulerAngles(
                            roll=0.5,
                            pitch=0.6,
                            yaw=0.7,
                        )
                    ),
                    present_speed=Vector3D(
                        x=self.head.neck.roll.present_speed,
                        y=self.head.neck.pitch.present_speed,
                        z=self.head.neck.yaw.present_speed,
                    ),
                    present_load=Vector3D(
                        x=self.head.neck.roll.present_load,
                        y=self.head.neck.pitch.present_load,
                        z=self.head.neck.yaw.present_load,
                    ),
                    compliant=BoolValue(value=self.head.neck.compliant),
                    goal_position=Rotation3D(
                        rpy=ExtEulerAngles(
                            roll=0.5,
                            pitch=0.6,
                            yaw=0.7,
                        )
                    ),
                    speed_limit=Float3D(
                        motor_1=self.head.neck.roll.speed_limit,
                        motor_2=self.head.neck.pitch.speed_limit,
                        motor_3=self.head.neck.yaw.speed_limit,
                    ),
                    torque_limit=Float3D(
                        motor_1=self.head.neck.roll.torque_limit,
                        motor_2=self.head.neck.pitch.torque_limit,
                        motor_3=self.head.neck.yaw.torque_limit,
                    ),
                    pid=PID3D(
                        motor_1=PIDGains(
                            p=self.head.neck.roll.pid.p,
                            i=self.head.neck.roll.pid.i,
                            d=self.head.neck.roll.pid.d,
                        ),
                        motor_2=PIDGains(
                            p=self.head.neck.pitch.pid.p,
                            i=self.head.neck.pitch.pid.i,
                            d=self.head.neck.pitch.pid.d,
                        ),
                        motor_3=PIDGains(
                            p=self.head.neck.yaw.pid.p,
                            i=self.head.neck.yaw.pid.i,
                            d=self.head.neck.yaw.pid.d,
                        ),
                    ),
                ),
                l_antenna_state=DynamixelMotorState(
                    id=ComponentId(
                        id=self.head.l_antenna.id,
                        name=self.head.l_antenna.name,
                    ),
                    temperature=self.head.l_antenna.pid.temperature,
                    present_position=self.head.l_antenna.pid.present_position,
                    present_speed=self.head.l_antenna.pid.present_speed,
                    present_load=self.head.l_antenna.pid.present_load,
                    compliant=BoolValue(value=self.head.l_antenna.compliant),
                    goal_position=self.head.l_antenna.pid.goal_position,
                    speed_limit=self.head.l_antenna.pid.speed_limit,
                    torque_limit=self.head.l_antenna.pid.torque_limit,
                    pid=PIDGains(
                            p=self.head.l_antenna.pid.p,
                            i=self.head.l_antenna.pid.i,
                            d=self.head.l_antenna.pid.d,
                        ),
                ),
                r_antenna_state=DynamixelMotorState(
                    id=ComponentId(
                        id=self.head.r_antenna.id,
                        name=self.head.r_antenna.name,
                    ),
                    temperature=self.head.r_antenna.pid.temperature,
                    present_position=self.head.r_antenna.pid.present_position,
                    present_speed=self.head.r_antenna.pid.present_speed,
                    present_load=self.head.r_antenna.pid.present_load,
                    compliant=BoolValue(value=self.head.r_antenna.compliant),
                    goal_position=self.head.r_antenna.pid.goal_position,
                    speed_limit=self.head.r_antenna.pid.speed_limit,
                    torque_limit=self.head.r_antenna.pid.torque_limit,
                    pid=PIDGains(
                            p=self.head.r_antenna.pid.p,
                            i=self.head.r_antenna.pid.i,
                            d=self.head.r_antenna.pid.d,
                        ),
                ),
            ),
            # l_hand_state=HandState(
            # ),
            # r_hand_state=HandState(
            # ),
            # mobile_base_state=MobileBaseState(
            # ),
        )

    def StreamReachyState(self, request: ReachyStreamStateRequest, context: grpc.ServicerContext) -> Iterator[ReachyState]:
        """Continuously stream requested joints up-to-date state."""
        dt = 1.0 / request.publish_frequency if request.publish_frequency > 0 else -1.0
        last_pub = 0.0

        while True:
            elapsed_time = time.time() - last_pub
            if elapsed_time < dt:
                time.sleep(dt - elapsed_time)

            reachy_state = self.GetReachyState(request.id, context)
            reachy_state.timestamp.GetCurrentTime()

            yield reachy_state
            last_pub = time.time()
