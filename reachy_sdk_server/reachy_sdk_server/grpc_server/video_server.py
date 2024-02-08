from threading import Lock
from typing import Dict, List

import cv2
import grpc
import numpy as np
import numpy.typing as npt
import rclpy
from depthai_wrappers.sdk_wrapper import SDKWrapper
from depthai_wrappers.utils import get_config_file_path, get_connected_devices
from google.protobuf.empty_pb2 import Empty
from google.protobuf.wrappers_pb2 import BoolValue
from reachy2_sdk_api.error_pb2 import Error
from reachy2_sdk_api.video_pb2 import CameraInfo, Frame, ListOfCameraInfo, VideoAck, View, ViewRequest
from reachy2_sdk_api.video_pb2_grpc import add_VideoServiceServicer_to_server


class ReachyGRPCVideoSDKServicer:
    def __init__(self) -> None:
        rclpy.init()
        self.node = rclpy.create_node("ReachyGRPCVideoSDKServicer_node")
        self._logger = self.node.get_logger()

        self._logger.info("Reachy GRPC Video SDK Servicer initialized.")

        self._available_cams: Dict[str, SDKWrapper] = {}
        self._list_cam: List[CameraInfo] = []
        self._captured_data: Dict[str, Dict[str, npt.NDArray[np.uint8]]] = {}

        self._nb_grpc_client = 0
        self._lock = Lock()

    def register_to_server(self, server: grpc.Server):
        self._logger.info("Registering 'VideoServiceServicer' to server.")
        add_VideoServiceServicer_to_server(self, server)

    def GoodBye(self, request: Empty, context: grpc.ServicerContext) -> Empty:
        self._logger.info(f"Client leaving. Remaining {self._nb_grpc_client}")
        with self._lock:
            self._nb_grpc_client -= 1
            if self._nb_grpc_client < 1:
                self._logger.info("No more client. Releasing cameras.")
                self._available_cams.clear()
                self._list_cam.clear()
                self._nb_grpc_client = 0
        return Empty()

    def InitAllCameras(self, request: CameraInfo, context: grpc.ServicerContext) -> ListOfCameraInfo:
        with self._lock:
            self._nb_grpc_client += 1
            if len(self._available_cams) != 0:
                self._logger.info("Cameras already initialized")
                return ListOfCameraInfo(camera_info=self._list_cam)

            self._logger.info("Initializing all cameras...")
            try:
                devices = get_connected_devices()
            except RuntimeError as e:
                self._logger.error(f"List of camera cannot be retrieved {e}.")
                return ListOfCameraInfo()

            self._list_cam = []
            for mxid, name in devices.items():
                ci = CameraInfo(mxid=mxid, name=name)
                if name == "other":  # hardcoded in pollen-vision
                    ci.stereo = False
                    ci.depth = True
                else:  # teleop otherwise
                    ci.stereo = True
                    ci.depth = False
                ack = self._init_camera(ci)
                if ack.success.value:
                    self._list_cam.append(ci)
                else:
                    self._logger.error(f"Error opening camera {ack.error}.")

        if len(self._list_cam) == 0:
            self._logger.debug("List of cam is empty")
            return ListOfCameraInfo()
        else:
            return ListOfCameraInfo(camera_info=self._list_cam)

    def _init_camera(self, camera_info: CameraInfo) -> VideoAck:
        try:
            if camera_info.name == "other":
                self._logger.info("Opening SR camera")
                cam = SDKWrapper(
                    get_config_file_path("CONFIG_SR"),
                    compute_depth=True,
                    rectify=False,
                    mx_id=camera_info.mxid,
                    jpeg_output=True,
                )
            else:
                self._logger.info("Opening teleop camera")
                cam = SDKWrapper(
                    get_config_file_path("CONFIG_IMX296"),
                    compute_depth=False,
                    rectify=True,
                    mx_id=camera_info.mxid,
                    jpeg_output=True,
                )
            self._available_cams[camera_info.mxid] = cam
            return VideoAck(success=BoolValue(value=True))
        except RuntimeError as e:
            return VideoAck(success=BoolValue(value=False), error=Error(details=str(e)))
        except Exception as e:
            return VideoAck(success=BoolValue(value=False), error=Error(details=str(e)))

    def GetFrame(self, request: ViewRequest, context: grpc.ServicerContext) -> Frame:
        """
        Frames are encoded in PNG to save bandwith
        """
        if request.camera_info.mxid not in self._available_cams:
            self._logger.warning(f"Camera {request.camera_info.mxid} not opened")
            return Frame(data=None)
        elif not request.camera_info.stereo and request.view == View.RIGHT:
            self._logger.warning(f"Camera {request.camera_info.mxid} has no stereo feature. Returning mono view")
        elif request.camera_info.mxid not in self._captured_data:
            self._logger.warning("No data captured. Make sure to call capture() first")
            return Frame(data=None)

        if not request.camera_info.stereo or request.view == View.LEFT:
            frame = self._captured_data[request.camera_info.mxid]["left"]
        else:
            frame = self._captured_data[request.camera_info.mxid]["right"]

        return Frame(data=frame.tobytes())

    def GetDepthFrame(self, request: ViewRequest, context: grpc.ServicerContext) -> Frame:
        if request.camera_info.mxid not in self._available_cams:
            self._logger.warning(f"Camera {request.camera_info.mxid} not opened")
            return Frame(data=None)
        elif not request.camera_info.depth:
            self._logger.warning(f"Camera {request.camera_info.mxid} has no depth feature")
            return Frame(data=None)
        elif request.camera_info.mxid not in self._captured_data:
            self._logger.warning("No data captured. Make sure to call capture() first")
            return Frame(data=None)

        res = False
        if request.camera_info.depth and request.view == View.LEFT:
            res, frame = cv2.imencode(".png", self._captured_data[request.camera_info.mxid]["depthNode_left"])
        elif request.camera_info.depth and request.view == View.RIGHT:
            res, frame = cv2.imencode(".png", self._captured_data[request.camera_info.mxid]["depthNode_right"])

        if res:
            return Frame(data=frame.tobytes())
        else:
            self._logger.error("Failed to encode image")
            return Frame(data=None)

    def GetDepthMap(self, request: CameraInfo, context: grpc.ServicerContext) -> Frame:
        if request.mxid not in self._available_cams:
            self._logger.warning(f"Camera {request.mxid} not opened")
            return Frame(data=None)
        elif not request.depth:
            self._logger.warning(f"Camera {request.mxid} has no depth feature")
            return Frame(data=None)
        elif request.mxid not in self._captured_data:
            self._logger.warning(f"No data captured. Make sure to call capture() first")
            return Frame(data=None)

        res, frame = cv2.imencode(".png", self._captured_data[request.mxid]["depth"])
        if res:
            return Frame(data=frame.tobytes())
        else:
            self._logger.error("Failed to encode image")
            return Frame(data=None)

    def GetDisparity(self, request: CameraInfo, context: grpc.ServicerContext) -> Frame:
        if request.mxid not in self._available_cams:
            self._logger.warning(f"Camera {request.mxid} not opened")
            return Frame(data=None)
        elif not request.depth:
            self._logger.warning(f"Camera {request.mxid} has no depth feature")
            return Frame(data=None)
        elif request.mxid not in self._captured_data:
            self._logger.warning(f"No data captured. Make sure to call capture() first")
            return Frame(data=None)

        res, frame = cv2.imencode(".png", self._captured_data[request.mxid]["disparity"])
        if res:
            return Frame(data=frame.tobytes())
        else:
            self._logger.error("Failed to encode image")
            return Frame(data=None)

    def Capture(self, request: CameraInfo, context: grpc.ServicerContext) -> VideoAck:
        self._logger.info(f"Capturing {request.mxid}")
        if request.mxid not in self._available_cams:
            return VideoAck(success=BoolValue(value=False), error=Error(details=f"Camera {request.mxid} not opened"))

        self._captured_data[request.mxid], _, _ = self._available_cams[request.mxid].get_data()
        return VideoAck(success=BoolValue(value=True))


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
