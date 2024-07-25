import threading
from enum import Enum
from functools import partial
from threading import Event, Lock
from typing import Dict, List, Optional

import cv2
import grpc
import numpy as np
import numpy.typing as npt
import rclpy
from google.protobuf.empty_pb2 import Empty
from google.protobuf.timestamp_pb2 import Timestamp
from google.protobuf.wrappers_pb2 import BoolValue
from pollen_vision.camera_wrappers.depthai import SDKWrapper
from pollen_vision.camera_wrappers.depthai.utils import get_config_file_path, get_connected_devices
from reachy2_sdk_api.error_pb2 import Error
from reachy2_sdk_api.video_pb2 import CameraFeatures, CameraParameters, Frame, ListOfCameraFeatures, VideoAck, View, ViewRequest
from reachy2_sdk_api.video_pb2_grpc import add_VideoServiceServicer_to_server
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg._compressed_image import CompressedImage


class CameraType(Enum):
    """Camera names defined in pollen-vision"""

    TELEOP = "teleop_head"
    DEPTH = "depth_camera"  # TODO


class ROSFrame:
    def __init__(self, timestamp, data):
        self.timestamp = timestamp
        self.data = data


class ROSCamInfo:
    def __init__(self, height, width, distortion_model, D, K, R, P):
        self.height = height
        self.width = width
        self.distortion_model = distortion_model
        self.D = D
        self.K = K
        self.R = R
        self.P = P


class ReachyGRPCVideoSDKServicer:
    def __init__(self) -> None:
        rclpy.init()
        self.node = rclpy.create_node("ReachyGRPCVideoSDKServicer_node")
        self._logger = self.node.get_logger()

        self._logger.info("Reachy GRPC Video SDK Servicer initialized.")

        self._list_cam = []
        # TODO add more cameras
        ci = CameraFeatures(name=CameraType.TELEOP.value, stereo=True, depth=False)
        self._list_cam.append(ci)

        self.left_camera_sub = self.node.create_subscription(
            CompressedImage,
            "teleop_camera/left_image/compressed",
            partial(self.on_image_update, side="left"),
            1,
        )

        self.right_camera_sub = self.node.create_subscription(
            CompressedImage,
            "teleop_camera/right_image/compressed",
            partial(self.on_image_update, side="right"),
            1,
        )

        self.cams_frame: Dict[str, Optional[ROSFrame]] = {"left": None, "right": None}

        self.left_camera_info_sub = self.node.create_subscription(
            CameraInfo,
            "teleop_camera/left_image/camera_info",
            partial(self.on_info_update, side="left"),
            1,
        )

        self.right_camera_info_sub = self.node.create_subscription(
            CameraInfo,
            "teleop_camera/right_image/camera_info",
            partial(self.on_info_update, side="right"),
            1,
        )

        self.cams_info: Dict[str, Optional[ROSCamInfo]] = {"left": None, "right": None}

        self.ros_thread = threading.Thread(target=self.spin_ros, daemon=True)
        self.ros_thread.start()

    def __del__(self) -> None:
        self.node.destroy_node()
        rclpy.shutdown()

    def spin_ros(self) -> None:
        self._logger.info("Spin node")
        rclpy.spin(self.node)

    def on_image_update(self, msg, side):
        """Get data from image. Callback for "/'side'_image "subscriber."""
        frame = ROSFrame(Timestamp(seconds=msg.header.stamp.sec, nanos=msg.header.stamp.nanosec), msg.data.tobytes())
        self.cams_frame[side] = frame

    def on_info_update(self, msg, side):
        """Get data from image. Callback for "/'side'_image "subscriber."""
        cam_info = ROSCamInfo(msg.height, msg.width, msg.distortion_model, msg.d, msg.k, msg.r, msg.p)
        self.cams_info[side] = cam_info

    def register_to_server(self, server: grpc.Server):
        self._logger.info("Registering 'VideoServiceServicer' to server.")
        add_VideoServiceServicer_to_server(self, server)

    def GetAvailableCameras(self, request: CameraInfo, context: grpc.ServicerContext) -> ListOfCameraFeatures:
        return ListOfCameraFeatures(camera_feat=self._list_cam)

    def GetFrame(self, request: ViewRequest, context: grpc.ServicerContext) -> Frame:
        """
        Frames are encoded in JPG to save bandwidth
        """
        if request.camera_feat.name not in [c.name for c in self._list_cam]:
            self._logger.warning(f"Camera {request.camera_info.name} not opened")
            return Frame(data=None, timestamp=None)
        elif not request.camera_feat.stereo and request.view == View.RIGHT:
            self._logger.warning(f"Camera {request.camera_info.name} has no stereo feature. Returning mono view")

        if not request.camera_feat.stereo or request.view == View.LEFT:
            frame = self.cams_frame["left"]
        else:
            frame = self.cams_frame["right"]

        if frame is None:
            self._logger.warning(f"No camera data published for {request.camera_feat.name}")
            return Frame(data=None, timestamp=None)

        return Frame(data=frame.data, timestamp=frame.timestamp)

    def GetParameters(self, request: ViewRequest, context: grpc.ServicerContext) -> CameraParameters:
        """
        Get camera parameters as defined in https://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/CameraInfo.html
        """
        if request.camera_feat.name not in [c.name for c in self._list_cam]:
            self._logger.warning(f"Camera {request.camera_feat.name} not opened")
            return CameraParameters()
        elif not request.camera_feat.stereo and request.view == View.RIGHT:
            self._logger.warning(f"Camera {request.camera_feat.name} has no stereo feature. Returning mono view")

        if not request.camera_feat.stereo or request.view == View.LEFT:
            cam_param = self.cams_info["left"]
        else:
            cam_param = self.cams_info["right"]

        if cam_param is None:
            self._logger.warning(f"No camera parameters published for {request.camera_feat.name}")
            return CameraParameters()

        return CameraParameters(
            height=cam_param.height,
            width=cam_param.width,
            distortion_model=cam_param.distortion_model,
            D=cam_param.D,
            K=cam_param.K,
            R=cam_param.R,
            P=cam_param.P,
        )

    """
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
        # self._logger.info(f"Capturing {request.mxid}")
        if request.mxid not in self._available_cams:
            return VideoAck(success=BoolValue(value=False), error=Error(details=f"Camera {request.mxid} not opened"))

        self._captured_data[request.mxid], _, _ = self._available_cams[request.mxid].get_data()
        return VideoAck(success=BoolValue(value=True))
    """


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
