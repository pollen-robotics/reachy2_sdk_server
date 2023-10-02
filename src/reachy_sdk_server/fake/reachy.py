from typing import Iterator
import grpc
import time

from google.protobuf.empty_pb2 import Empty

from reachy_sdk_api_v2.arm_pb2 import Arm, ArmDescription, ArmState, ArmPosition
from reachy_sdk_api_v2.head_pb2 import HeadState
from reachy_sdk_api_v2.hand_pb2 import HandState
from reachy_sdk_api_v2.mobile_base_pb2 import MobileBaseState
from reachy_sdk_api_v2.component_pb2 import ComponentId, PIDGains
from reachy_sdk_api_v2.orbita2d_pb2 import Orbita2DInfo, Axis, Orbita2DState, Float2D, PID2D
from reachy_sdk_api_v2.orbita3d_pb2 import Orbita3DInfo, Orbita3DState, Float3D, PID3D
from reachy_sdk_api_v2.part_pb2 import PartId, PartInfo
from reachy_sdk_api_v2.reachy_pb2_grpc import ReachyServiceServicer
from reachy_sdk_api_v2.reachy_pb2 import Reachy, ReachyId
from reachy_sdk_api_v2.reachy_pb2 import ReachyStreamStateRequest, ReachyState

class ReachyServicer(ReachyServiceServicer):
    def __init__(self, right_arm) -> None:
        self.right_arm = right_arm
        self._temp = 0.01

    def GetReachy(self, request: Empty, context) -> Reachy:
        return Reachy(
            r_arm=Arm(
                part_id=PartId(
                    name=self.right_arm.name,
                ),
                description=ArmDescription(
                    shoulder=Orbita2DInfo(
                        id=ComponentId(
                            id=self.right_arm.shoulder.id
                            ),
                        serial_number=self.right_arm.shoulder.serial_number,
                        axis_1=getattr(Axis, self.right_arm.shoulder._axis1_type.upper()),
                        axis_2=getattr(Axis, self.right_arm.shoulder._axis2_type.upper()),
                    ),
                    elbow=Orbita2DInfo(
                        id=ComponentId(
                            id=self.right_arm.elbow.id
                            ),
                        serial_number=self.right_arm.elbow.serial_number,
                        axis_1=getattr(Axis, self.right_arm.elbow._axis1_type.upper()),
                        axis_2=getattr(Axis, self.right_arm.elbow._axis2_type.upper()),
                    ),
                    wrist=Orbita3DInfo(
                        id=ComponentId(
                            id=self.right_arm.wrist.id
                            ),
                        serial_number=self.right_arm.wrist.serial_number,
                    ),
                )
            )
        )

    def GetReachyState(self, uid: ReachyId, context: grpc.ServicerContext) -> ReachyState:
        """Get the requested joints id."""
        self._temp += 0.01
        return ReachyState(
            # l_arm_state=ArmState(
            #     name=self.left_arm.name

            # ),
            r_arm_state=ArmState(
                name=self.right_arm.name,
                shoulder_state=Orbita2DState(
                    name=self.right_arm.shoulder.id,
                    temperature=Float2D(
                        axis_1=self._temp,
                        axis_2=0.6
                    ),
                    present_position=Float2D(
                        axis_1=self._temp,
                        axis_2=0.6
                    ),
                    present_speed=Float2D(
                        axis_1=self._temp,
                        axis_2=0.6
                    ),
                    present_load=Float2D(
                        axis_1=self._temp,
                        axis_2=0.6
                    ),
                    goal_position=Float2D(
                        axis_1=self._temp,
                        axis_2=0.6
                    ),
                    speed_limit=Float2D(
                        axis_1=self._temp,
                        axis_2=0.6
                    ),
                    torque_limit=Float2D(
                        axis_1=self._temp,
                        axis_2=0.6
                    ),
                    pid=PID2D(
                        gains_axis_1=PIDGains(
                            p=500,
                            i=200,
                            d=3,
                        ),
                        gains_axis_2=PIDGains(
                            p=400,
                            i=700,
                            d=4,
                        )
                    ),
                ),
                elbow_state=Orbita2DState(
                    name=self.right_arm.elbow.id,
                    temperature=Float2D(
                        axis_1=self._temp,
                        axis_2=0.6
                    ),
                    present_position=Float2D(
                        axis_1=self._temp,
                        axis_2=0.6
                    ),
                    present_speed=Float2D(
                        axis_1=self._temp,
                        axis_2=0.6
                    ),
                    present_load=Float2D(
                        axis_1=self._temp,
                        axis_2=0.6
                    ),
                    goal_position=Float2D(
                        axis_1=self._temp,
                        axis_2=0.6
                    ),
                    speed_limit=Float2D(
                        axis_1=self._temp,
                        axis_2=0.6
                    ),
                    torque_limit=Float2D(
                        axis_1=self._temp,
                        axis_2=0.6
                    ),
                    pid=PID2D(
                        gains_axis_1=PIDGains(
                            p=500,
                            i=200,
                            d=3,
                        ),
                        gains_axis_2=PIDGains(
                            p=400,
                            i=700,
                            d=4,
                        )
                    ),
                ),
                wrist_state=Orbita2DState(
                    name=self.right_arm.wrist.id,
                    temperature=Float3D(
                        roll=self.temp,
                        pitch=0.6,
                        yaw=0.7,
                    ),
                    present_position=Float3D(
                        roll=self.temp,
                        pitch=0.6,
                        yaw=0.7,
                    ),
                    present_speed=Float3D(
                        roll=self.temp,
                        pitch=0.6,
                        yaw=0.7,
                    ),
                    present_load=Float3D(
                        roll=self.temp,
                        pitch=0.6,
                        yaw=0.7,
                    ),
                    goal_position=Float3D(
                        roll=self.temp,
                        pitch=0.6,
                        yaw=0.7,
                    ),
                    speed_limit=Float3D(
                        roll=self.temp,
                        pitch=0.6,
                        yaw=0.7,
                    ),
                    torque_limit=Float3D(
                        roll=self.temp,
                        pitch=0.6,
                        yaw=0.7,
                    ),
                    pid=PID3D(
                        roll=PIDGains(
                            p=500,
                            i=200,
                            d=3,
                        ),
                        pitch=PIDGains(
                            p=400,
                            i=700,
                            d=4,
                        ),
                        yaw=PIDGains(
                            p=200,
                            i=100,
                            d=1,
                        ),
                    ),
                ),
            ),
            # head_state=HeadState(
            # ),
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
