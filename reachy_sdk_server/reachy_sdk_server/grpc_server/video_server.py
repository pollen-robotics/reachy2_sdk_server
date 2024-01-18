from typing import List

import grpc
import rclpy
from depthai_wrappers.utils import get_connected_devices
from google.protobuf.empty_pb2 import Empty
from google.protobuf.wrappers_pb2 import BoolValue
from reachy2_sdk_api.component_pb2 import ComponentId
from reachy2_sdk_api.video_pb2 import CameraInfo, ListOfCameraInfo
from reachy2_sdk_api.video_pb2_grpc import add_VideoServiceServicer_to_server


class ReachyGRPCVideoSDKServicer:
    def __init__(self) -> None:
        rclpy.init()
        self.node = rclpy.create_node("ReachyGRPCVideoSDKServicer_node")

        self.node.get_logger().info("Reachy GRPC Video SDK Servicer initialized.")

    def register_to_server(self, server: grpc.Server):
        self.node.get_logger().info("Registering 'VideoServiceServicer' to server.")
        add_VideoServiceServicer_to_server(self, server)

    def GetAllCameras(
        self, request: Empty, context: grpc.ServicerContext
    ) -> ListOfCameraInfo:
        self.node.get_logger().info("fetching camera info...")
        devices = get_connected_devices()
        self.node.get_logger().info("list of camera info retrived.")
        list_cam: List[CameraInfo] = []
        for mxid, name in devices.items():
            ci = CameraInfo(
                stereo=BoolValue(value=False),
                depth=BoolValue(value=False),
                id=ComponentId(id=mxid, name=name),
            )
            if name == "other":
                ci.stereo = BoolValue(value=False)
                ci.depth = BoolValue(value=True)
            else:
                ci.stereo = BoolValue(value=True)
                ci.depth = BoolValue(value=False)
            list_cam.append(ci)

        return ListOfCameraInfo(list_cam)

    def GetFrame(self, request: ComponentId, context: grpc.ServicerContext):
        self.node.get_logger().debug(f"get frame {request.id}")
        pass


def main():
    import argparse
    from concurrent import futures

    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=50065)
    parser.add_argument("--max-workers", type=int, default=10)
    parser.add_argument("--ros-args", action="store_true")
    args = parser.parse_args()

    servicer = ReachyGRPCVideoSDKServicer()
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=args.max_workers))

    servicer.register_to_server(server)

    server.add_insecure_port(f"[::]:{args.port}")
    server.start()

    servicer.node.get_logger().info(f"Server started on port {args.port}.")
    server.wait_for_termination()


if __name__ == "__main__":
    main()
