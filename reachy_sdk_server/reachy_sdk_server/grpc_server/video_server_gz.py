from threading import Lock, Event
import threading
from typing import Dict, List, Tuple
from asyncio.events import AbstractEventLoop
import asyncio
import cv2
import grpc
import numpy as np
import numpy.typing as npt
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time

from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo as CameraInfoMsg
from cv_bridge import CvBridge, CvBridgeError

# from depthai_wrappers.sdk_wrapper import SDKWrapper
# from depthai_wrappers.utils import get_config_file_path, get_connected_devices
from google.protobuf.empty_pb2 import Empty
from google.protobuf.wrappers_pb2 import BoolValue
from reachy2_sdk_api.error_pb2 import Error
from reachy2_sdk_api.video_pb2 import CameraInfo, Frame, ListOfCameraInfo, VideoAck, View, ViewRequest, IntrinsicMatrix
from reachy2_sdk_api.video_pb2_grpc import add_VideoServiceServicer_to_server
import time
from datetime import timedelta


class RosCamWrapper:
    def __init__(self, mxid, name) -> None:
        self._mxid = mxid
        self._name = name
        self._data = {}
        self._latency = {}
        self._ts = {}
        self._K = {}

    # def push_data(self, data:Dict[str, npt.NDArray[np.uint8]], latency: Dict[str, float], ts:Dict[str, timedelta]) -> None:
    #     self._data=data
    #     self._latency=latency
    #     self._ts=ts

    def get_data(self) -> Tuple[Dict[str, npt.NDArray[np.uint8]], Dict[str, float], Dict[str, timedelta]]:
        return self._data, self._latency, self._ts

    # fixme, dict with all cameras
    def get_K(self) -> Dict[str, npt.NDArray[np.float64]]:
        return self._K


class RosCameraNodes(Node):
    def __init__(self, asyncio_loop: AbstractEventLoop = None) -> None:
        super().__init__(node_name="reachy_gz_camera_node")
        self.logger = self.get_logger()
        self.logger.warning("RosCameraNodes")
        self.asyncio_loop = asyncio_loop

        self.depth_image_topic = "/depth_camera/depth/image_rect_raw"
        self.depth_info_topic = "/depth_camera/depth/camera_info"

        self.l_rgb_image_topic = "/sr_left_image/image_raw"
        self.l_rgb_info_topic = "/sr_left_image/camera_info"

        self.r_rgb_image_topic = "/sr_right_image/image_raw"
        self.r_rgb_info_topic = "/sr_right_image/camera_info"

        self.got_depth = Event()
        self.got_l_rgb = Event()
        self.got_r_rgb = Event()

        self.bridge = CvBridge()
        self.depth_sub = self.create_subscription(Image, self.depth_image_topic, self.imageDepthCallback, 1)
        self.depth_info_sub = self.create_subscription(CameraInfoMsg, self.depth_info_topic, self.imageDepthInfoCallback, 1)

        self.l_rgb_sub = self.create_subscription(Image, self.l_rgb_image_topic, self.imageLRGBCallback, 1)
        self.l_rgb_info_sub = self.create_subscription(CameraInfoMsg, self.l_rgb_info_topic, self.imageLRGBInfoCallback, 1)

        self.r_rgb_sub = self.create_subscription(Image, self.r_rgb_image_topic, self.imageRRGBCallback, 1)
        self.r_rgb_info_sub = self.create_subscription(CameraInfoMsg, self.r_rgb_info_topic, self.imageRRGBInfoCallback, 1)

        self.curr_depth = None
        self.curr_l_rgb = None
        self.curr_r_rgb = None

        self.depth_info = None
        self.l_rgb_info = None
        self.r_rgb_info = None

        self.prev_depth_ts = self.get_clock().now()
        self.curr_depth_ts = self.get_clock().now()

        self.prev_l_rgb_ts = self.get_clock().now()
        self.curr_l_rgb_ts = self.get_clock().now()

        self.prev_r_rgb_ts = self.get_clock().now()
        self.curr_r_rgb_ts = self.get_clock().now()

        self._cams: Dict[str, str] = {}

        self._available_cams = None

    def push_data(self, name: str, img: npt.NDArray[np.uint8], latency: float, ts: timedelta) -> None:
        if self._available_cams is not None:
            if self._available_cams["gz_camera"] is not None:
                self._available_cams["gz_camera"]._data[name] = img
                self._available_cams["gz_camera"]._latency[name] = latency
                self._available_cams["gz_camera"]._ts[name] = ts
                # self.logger.warning(f"PUSH DATA")

    def push_K(self, name: str, K: npt.NDArray[np.float64]) -> None:
        # self.logger.warning(f"PUSHK")
        if self._available_cams is not None:
            if self._available_cams["gz_camera"] is not None:
                self._available_cams["gz_camera"]._K[name] = K
                # self.logger.warning(f"YO")

    # def get_data(self)-> Tuple[Dict[str, npt.NDArray[np.uint8]], Dict[str, float], Dict[str, timedelta]]:

    #     data: Dict[str, npt.NDArray[np.uint8]] = {}
    #     latency: Dict[str, float] = {}
    #     ts: Dict[str, timedelta] = {}

    #     data['left']=self.curr_l_rgb
    #     data['right']=self.curr_r_rgb
    #     data['depth']=self.curr_depth

    #     latency['left']=0.0
    #     latency['right']=0.0
    #     latency['depth']=0.0

    #     ts['left']=self.curr_depth_ts-self.prev_depth_ts
    #     ts['right']=self.curr_r_rgb_ts-self.prev_r_rgb_ts
    #     ts['depth']=self.curr_l_rgb_ts-self.prev_l_rgb_ts

    #     return data, latency, ts

    def wait_for_ready(self) -> None:
        # Wait
        while not (self.got_depth.is_set() and self.got_l_rgb.is_set() and self.got_r_rgb.is_set()):
            rclpy.spin_once(self)
            time.sleep(1)

    def imageDepthCallback(self, data: Image) -> None:
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
            #convert to uint16 in mm for compatibility with Luxonis SR : This is stupid
            cv_image*=1000.0
            cv_image=cv_image.astype(np.uint16)
            self.curr_depth = cv_image
            self.prev_depth_ts = self.curr_depth_ts
            self.curr_depth_ts = Time.from_msg(data.header.stamp)
            self.push_data("depth", self.curr_depth, 0.0, self.curr_depth_ts - self.prev_depth_ts)

        except CvBridgeError as e:
            self.logger.error(f"{e}")
            return
        except ValueError as e:
            return

    def imageLRGBCallback(self, data: Image) -> None:
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
            self.curr_l_rgb = cv_image
            self.prev_l_rgb_ts = self.curr_l_rgb_ts
            self.curr_l_rgb_ts = Time.from_msg(data.header.stamp)
            self.push_data("left", self.curr_l_rgb, 0.0, self.curr_l_rgb_ts - self.prev_l_rgb_ts)

        except CvBridgeError as e:
            self.logger.error(f"{e}")
            return
        except ValueError as e:
            return

    def imageRRGBCallback(self, data: Image) -> None:
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
            self.curr_r_rgb = cv_image
            self.prev_r_rgb_ts = self.curr_r_rgb_ts
            self.curr_r_rgb_ts = Time.from_msg(data.header.stamp)
            self.push_data("right", self.curr_r_rgb, 0.0, self.curr_r_rgb_ts - self.prev_r_rgb_ts)

        except CvBridgeError as e:
            self.logger.error(f"{e}")
            return
        except ValueError as e:
            return

    def imageDepthInfoCallback(self, cameraInfo: CameraInfoMsg) -> None:
        self.depth_info = cameraInfo
        self.got_depth.set()
        # self._cams[self.depth_image_topic]='other'
        self._cams["gz_camera"] = "other"

        try:
            self.push_K("depth", cameraInfo.k)
        except Exception as err:
            self.logger.error(f"Depth Info: {err}")

    def imageLRGBInfoCallback(self, cameraInfo: CameraInfoMsg) -> None:
        self.l_rgb_info = cameraInfo
        self.got_l_rgb.set()
        # self._cams[self.l_rgb_image_topic]='other'
        self._cams["gz_camera"] = "other"
        try:
            self.push_K("left", cameraInfo.k)
        except Exception as err:
            self.logger.error(f"Left Info: {err}")

    def imageRRGBInfoCallback(self, cameraInfo: CameraInfoMsg) -> None:
        self.r_rgb_info = cameraInfo
        self.got_r_rgb.set()
        # self._cams[self.r_rgb_image_topic]='other'
        self._cams["gz_camera"] = "other"
        try:
            self.push_K("right", cameraInfo.k)
        except Exception as err:
            self.logger.error(f"Right Info: {err}")


class ReachyGRPCVideoSDKServicer:
    def __init__(self) -> None:
        print("DEBUG GRPC VIDEO")
        rclpy.init()

        self.asyncio_loop = asyncio.new_event_loop()

        self.bridge_node = RosCameraNodes(asyncio_loop=self.asyncio_loop)

        self.asyncio_thread = threading.Thread(target=self.spin_asyncio)
        self.asyncio_thread.start()

        # rclpy.init()
        # self.node = rclpy.create_node("ReachyGRPCVideoSDKServicer_node")
        self._logger = self.bridge_node.get_logger()

        self._logger.info("Reachy GRPC Video SDK Servicer initialized.")

        self._available_cams: Dict[str, RosCamWrapper] = {}

        self._list_cam: List[CameraInfo] = []

        self._captured_data: Dict[str, Dict[str, npt.NDArray[np.uint8]]] = {}
        self._K: Dict[str, Dict[str, npt.NDArray[np.float64]]] = {}

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
                self.bridge_node._available_cams = None
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
                # devices = get_connected_devices()
                # self.bridge_node.wait_for_ready()
                self._logger.info("Cameras initialized")
                devices = self.bridge_node._cams
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
                # self._list_cam.append(ci)
                else:
                    self._logger.error(f"Error opening camera {ack.error}.")

        if len(self._list_cam) == 0:
            self._logger.debug("List of cam is empty")
            return ListOfCameraInfo()
        else:
            return ListOfCameraInfo(camera_info=self._list_cam)

    def _init_camera(self, camera_info: CameraInfo) -> VideoAck:
        self._logger.warning(f"INIT CAMERA {camera_info}")
        try:
            if camera_info.name == "other":
                self._logger.info("Opening SR camera")
                # cam = SDKWrapper(
                #     get_config_file_path("CONFIG_SR"),
                #     compute_depth=True,
                #     rectify=False,
                #     mx_id=camera_info.mxid,
                #     jpeg_output=True,
                # )
                cam = RosCamWrapper("gz_camera", "SR")
            else:
                self._logger.info("Opening teleop camera: TODO")
                # cam = SDKWrapper(
                #     get_config_file_path("CONFIG_IMX296"),
                #     compute_depth=False,
                #     rectify=True,
                #     mx_id=camera_info.mxid,
                #     jpeg_output=True,
                # )
                # cam=RosCamWrapper("teleop")

            self._available_cams[camera_info.mxid] = cam
            self._logger.warning(f"available cams: {self._available_cams}")
            self.bridge_node._available_cams = self._available_cams
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

        res = False
        # if not request.camera_info.stereo or request.view == View.LEFT:
        if request.view == View.LEFT:
            res, frame = cv2.imencode(".png", self._captured_data[request.camera_info.mxid]["left"])
        else:
            res, frame = cv2.imencode(".png", self._captured_data[request.camera_info.mxid]["right"])

        if res:
            return Frame(data=frame.tobytes())
        else:
            self._logger.error("Failed to encode image")
            return Frame(data=None)

    def GetIntrinsicMatrix(self, request: ViewRequest, context: grpc.ServicerContext) -> IntrinsicMatrix:
        """
        Get the intrinsic matrix K for the left (or right) camera
        """
        if request.camera_info.mxid not in self._available_cams:
            self._logger.warning(f"Camera {request.camera_info.mxid} not opened")
            return IntrinsicMatrix(fx=None, fy=None, cx=None, cy=None)

        elif request.camera_info.mxid not in self._K:
            self._logger.warning("No data captured. Make sure to call capture() first")
            return IntrinsicMatrix(fx=None, fy=None, cx=None, cy=None)

        if not request.camera_info.stereo or request.view == View.LEFT:
            intrinsic = self._K[request.camera_info.mxid]["left"]

        intrinsic = intrinsic.reshape((3, 3))

        return IntrinsicMatrix(fx=intrinsic[0][0], fy=intrinsic[1][1], cx=intrinsic[0][2], cy=intrinsic[1][2])

    def GetDepthIntrinsicMatrix(self, request: CameraInfo, context: grpc.ServicerContext) -> IntrinsicMatrix:
        """
        Get the intrinsic matrix K for the depth camera
        """
        if request.mxid not in self._available_cams:
            self._logger.warning(f"Camera {request.mxid} not opened")
            return IntrinsicMatrix(fx=None, fy=None, cx=None, cy=None)

        elif not request.depth:
            self._logger.warning(f"Camera {request.mxid} has no depth feature")
            return IntrinsicMatrix(fx=None, fy=None, cx=None, cy=None)

        elif request.mxid not in self._captured_data:
            self._logger.warning(f"No data captured. Make sure to call capture() first")
            return IntrinsicMatrix(fx=None, fy=None, cx=None, cy=None)

        intrinsic = self._K[request.mxid]["depth"]
        intrinsic = intrinsic.reshape((3, 3))

        return IntrinsicMatrix(fx=intrinsic[0][0], fy=intrinsic[1][1], cx=intrinsic[0][2], cy=intrinsic[1][2])

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

        # self._logger.warning(f"captured: {self._captured_data}")

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
        self._K[request.mxid] = self._available_cams[request.mxid].get_K()  # fixme left-right?
        return VideoAck(success=BoolValue(value=True))

    def spin_asyncio(self) -> None:
        asyncio.set_event_loop(self.asyncio_loop)
        self.asyncio_loop.run_until_complete(self.spinning(self.bridge_node))

    async def spinning(self, node):
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)

            await asyncio.sleep(0.001)


def main():
    import argparse
    from concurrent import futures

    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=50065)
    parser.add_argument("--max-workers", type=int, default=10)
    parser.add_argument("--ros-args", action="store_true")
    args = parser.parse_args()

    print("DEBUG: STARING GZ VIDEO")
    servicer = ReachyGRPCVideoSDKServicer()

    server = grpc.server(futures.ThreadPoolExecutor(max_workers=args.max_workers))

    servicer.register_to_server(server)

    server.add_insecure_port(f"[::]:{args.port}")
    server.start()

    servicer._logger.info(f"Server started on port {args.port}.")
    server.wait_for_termination()


if __name__ == "__main__":
    main()
