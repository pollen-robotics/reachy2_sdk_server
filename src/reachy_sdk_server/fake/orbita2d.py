from typing import Iterator
from google.protobuf.empty_pb2 import Empty
from google.protobuf.timestamp_pb2 import Timestamp
from grpc import ServicerContext
from reachy_sdk_api_v2.orbita2d_pb2 import (
    ListOfOrbita2DInfo,
    Orbita2DCommand,
    Orbita2DInfo,
    Orbita2DState,
    Orbita2DStateRequest,
    Orbita2DStreamStateRequest,
)
from reachy_sdk_api_v2.orbita2d_pb2_grpc import Orbita2DServiceServicer

from .component import (
    check_component_id,
    get_component,
    get_all_components_by_type,
    make_random_component,
)
from .utils import endless_get_stream


class Orbita2D(Orbita2DServiceServicer):
    def __init__(self) -> None:
        for _ in range(4):
            make_random_component(Orbita2DInfo, "orbita2d")

    def GetAllOrbita2D(
        self, request: Empty, context: ServicerContext
    ) -> ListOfOrbita2DInfo:
        return ListOfOrbita2DInfo(info=get_all_components_by_type(Orbita2DInfo))

    def GetState(
        self, request: Orbita2DStateRequest, context: ServicerContext
    ) -> Orbita2DState:
        check_component_id(request.id, "orbita2d", context)

        c = get_component(request.id)

        timestamp = Timestamp()
        timestamp.GetCurrentTime()

        return Orbita2DState(
            # fill up state with c values
            timestamp=timestamp,
        )

    def StreamState(
        self, request: Orbita2DStreamStateRequest, context: ServicerContext
    ):
        return endless_get_stream(
            self.GetState, request.req, context, period=1 / request.freq
        )

    def SendCommand(self, request: Orbita2DCommand, context: ServicerContext):
        check_component_id(request.id, "orbita2d", context)

        c = get_component(request.id)
        # TODO: apply command

        return super().SendCommand(request, context)

    def StreamCommand(
        self, request_iterator: Iterator[Orbita2DCommand], context: ServicerContext
    ):
        for request in request_iterator:
            self.SendCommand(request, context)

    def Audit(self, request, context: ServicerContext):
        check_component_id(request.id, "orbita2d", context)

        return super().Audit(request, context)

    def HeartBeat(self, request, context: ServicerContext):
        check_component_id(request.id, "orbita2d", context)

        return super().HeartBeat(request, context)

    def Restart(self, request, context: ServicerContext):
        check_component_id(request.id, "orbita2d", context)

        return super().Restart(request, context)
