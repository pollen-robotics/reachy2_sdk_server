from google.protobuf.empty_pb2 import Empty

from reachy_sdk_api_v2.arm_pb2 import Arm, ArmDescription
from reachy_sdk_api_v2.head_pb2 import Head, HeadDescription
from reachy_sdk_api_v2.component_pb2 import ComponentId
from reachy_sdk_api_v2.orbita2d_pb2 import Orbita2DInfo, Axis
from reachy_sdk_api_v2.orbita3d_pb2 import Orbita3DInfo
from reachy_sdk_api_v2.dynamixel_motor_pb2 import DynamixelMotorInfo
from reachy_sdk_api_v2.part_pb2 import PartId, PartInfo
from reachy_sdk_api_v2.reachy_pb2_grpc import ReachyServiceServicer
from reachy_sdk_api_v2.reachy_pb2 import Reachy

class ReachyServicer(ReachyServiceServicer):
    def __init__(self, right_arm, head) -> None:
        self.right_arm = right_arm
        self.head = head

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
            ),
            head=Head(
                part_id=PartId(
                    name=self.head.name,
                ),
                description=HeadDescription(
                    neck=Orbita3DInfo(
                        id=ComponentId(
                            id=self.head.neck.id
                            ),
                        serial_number=self.head.neck.serial_number,
                    ),
                    l_antenna=DynamixelMotorInfo(
                        id=ComponentId(
                            id=self.head.l_antenna.id
                            ),
                        serial_number=self.head.l_antenna.serial_number,
                    ),
                    r_antenna=DynamixelMotorInfo(
                        id=ComponentId(
                            id=self.head.r_antenna.id
                            ),
                        serial_number=self.head.r_antenna.serial_number,
                    ),
                )
            )
        )
