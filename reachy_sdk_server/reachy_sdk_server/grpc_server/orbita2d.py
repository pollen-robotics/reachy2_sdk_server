from typing import Iterator
import grpc
from reachy_sdk_api_v2.component_pb2 import ComponentId
import rclpy

from reachy_sdk_api_v2.orbita2d_pb2_grpc import add_Orbita2DServiceServicer_to_server

from ..abstract_bridge_node import AbstractBridgeNode
from ..utils import axis_from_str

from google.protobuf.empty_pb2 import Empty

from reachy_sdk_api_v2.orbita2d_pb2 import (
    ListOfOrbita2DInfo,
    Orbita2DCommand,
    Orbita2DInfo,
    Orbita2DState,
    Orbita2DStateRequest,
    Orbita2DStatus,
    Orbita2DStreamStateRequest,
)


class Orbita2dServicer:
    def __init__(
        self,
        bridge_node: AbstractBridgeNode,
        logger: rclpy.impl.rcutils_logger.RcutilsLogger,
    ) -> None:
        self.bridge_node = bridge_node
        self.logger = logger

    def register_to_server(self, server: grpc.Server):
        self.logger.info("Registering 'Orbita2dServiceServicer' to server.")
        add_Orbita2DServiceServicer_to_server(self, server)

    def GetAllOrbita2D(
        self, request: Empty, context: grpc.ServicerContext
    ) -> ListOfOrbita2DInfo:
        orbita2d = self.bridge_node.get_all_orbita2ds()

        infos = ListOfOrbita2DInfo(
            info=[
                Orbita2DInfo(
                    id=ComponentId(
                        id=o.id,
                        name=o.name,
                    ),
                    axis_1=axis_from_str(o.extra["axis1"]),
                    axis_2=axis_from_str(o.extra["axis2"]),
                )
                for o in orbita2d
            ]
        )
        return infos

    # State
    def GetState(
        self, request: Orbita2DStateRequest, context: grpc.ServicerContext
    ) -> Orbita2DState:
        return Orbita2DState()

    def StreamState(
        self, request: Orbita2DStreamStateRequest, context: grpc.ServicerContext
    ) -> Iterator[Orbita2DState]:
        yield Orbita2DState()

    # Command
    def SendCommand(
        self, request: Orbita2DCommand, context: grpc.ServicerContext
    ) -> Empty:
        return Empty()

    def StreamCommand(
        self, request_stream: Iterator[Orbita2DCommand], context: grpc.ServicerContext
    ) -> Empty:
        return Empty()

    # Doctor
    def Audit(
        self, request: ComponentId, context: grpc.ServicerContext
    ) -> Orbita2DStatus:
        return Orbita2DStatus()

    def HeartBeat(self, request: ComponentId, context: grpc.ServicerContext) -> Empty:
        return Empty()

    def Restart(self, request: ComponentId, context: grpc.ServicerContext) -> Empty:
        return Empty()
