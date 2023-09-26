import random
import uuid

from google.protobuf.empty_pb2 import Empty

from reachy_sdk_api_v2.component_pb2 import ComponentId
from reachy_sdk_api_v2 import orbita2d_pb2, orbita2d_pb2_grpc
from reachy_sdk_api_v2.orbita2d_pb2 import (
    ListOfOrbita2DInfo,
    Orbita2DStateRequest,
    Orbita2DState,
)


def make_random_orbita2d_info():
    id = f"orbita2d-{uuid.uuid4()}"
    serial_number = uuid.uuid4().hex

    return orbita2d_pb2.Orbita2DInfo(
        id=ComponentId(id=id),
        serial_number=serial_number,
    )


class Orbita2D(orbita2d_pb2_grpc.Orbita2DServiceServicer):
    def __init__(self) -> None:
        self.orbita2d = [
            make_random_orbita2d_info() for _ in range(random.randint(1, 5))
        ]

    def GetAllOrbita2D(self, request: Empty, context) -> ListOfOrbita2DInfo:
        return ListOfOrbita2DInfo(info=self.orbita2d)

    def GetState(self, request: Orbita2DStateRequest, context) -> Orbita2DState:
        return Orbita2DState()

    def StreamState(self, request, context):
        return super().StreamState(request, context)

    def SendCommand(self, request, context):
        return super().SendCommand(request, context)

    def StreamCommand(self, request, context):
        return super().StreamCommand(request, context)

    def Audit(self, request, context):
        return super().Audit(request, context)

    def HeartBeat(self, request, context):
        return super().HeartBeat(request, context)

    def Restart(self, request, context):
        return super().Restart(request, context)
