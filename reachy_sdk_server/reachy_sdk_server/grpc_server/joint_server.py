import asyncio
import threading

import grpc
import rclpy

from ..abstract_bridge_node import AbstractBridgeNode
from .arm import ArmServicer
from .goto import GoToServicer
from .hand import HandServicer
from .head import HeadServicer
from .mobile_base import MobileBaseServicer
from .orbita2d import Orbita2dServicer
from .orbita3d import Orbita3dServicer
from .reachy import ReachyServicer


class ReachyGRPCJointSDKServicer:
    def __init__(self, reachy_config_path: str = None) -> None:
        rclpy.init()

        # executor = rclpy.executors.MultiThreadedExecutor()
        # executor.add_node(self.bridge_node)
        # threading.Thread(target=executor.spin).start()

        # self.asyncio_loop = asyncio.get_event_loop()
        self.asyncio_loop = asyncio.new_event_loop()

        # Should we have separate nodes for each servicer instead?
        self.bridge_node = AbstractBridgeNode(reachy_config_path=reachy_config_path, asyncio_loop=self.asyncio_loop)

        self.asyncio_thread = threading.Thread(target=self.spin_asyncio)
        self.asyncio_thread.start()

        self.asyncio_loop2 = asyncio.new_event_loop()
        self.asyncio_thread2 = threading.Thread(target=self.spin_asyncio2)
        self.asyncio_thread2.start()

        self.logger = self.bridge_node.get_logger()

        orbita2d_servicer = Orbita2dServicer(self.bridge_node, self.logger)
        orbita3d_servicer = Orbita3dServicer(self.bridge_node, self.logger)
        arm_servicer = ArmServicer(
            self.bridge_node,
            self.logger,
            orbita2d_servicer,
            orbita3d_servicer,
        )
        goto_servicer = GoToServicer(self.bridge_node, self.logger)
        hand_servicer = HandServicer(self.bridge_node, self.logger)
        head_servicer = HeadServicer(self.bridge_node, self.logger, orbita3d_servicer)
        mobile_base_servicer = MobileBaseServicer(self.bridge_node, self.logger, reachy_config_path)
        reachy_servicer = ReachyServicer(
            self.bridge_node,
            self.logger,
            arm_servicer,
            hand_servicer,
            head_servicer,
            mobile_base_servicer,
        )

        self.services = [
            arm_servicer,
            goto_servicer,
            hand_servicer,
            head_servicer,
            mobile_base_servicer,
            orbita2d_servicer,
            orbita3d_servicer,
            reachy_servicer,
        ]

        self.logger.info("Reachy GRPC Joint SDK Servicer initialized.")

    def register_all_services(self, server: grpc.Server) -> None:
        for serv in self.services:
            serv.register_to_server(server)

    def spin_asyncio(self) -> None:
        asyncio.set_event_loop(self.asyncio_loop)
        self.asyncio_loop.run_until_complete(self.spinning(self.bridge_node))

    def spin_asyncio2(self) -> None:
        asyncio.set_event_loop(self.asyncio_loop2)
        self.asyncio_loop2.run_until_complete(self.spinning2(self.bridge_node))

    async def spinning2(self, node):
        with node.sum_spin2.time():
            await asyncio.sleep(1)

    async def spinning(self, node):
        while rclpy.ok():
            with node.sum_spin.time():
                rclpy.spin_once(node, timeout_sec=0.01)
            await asyncio.sleep(0.001)


def main():
    import argparse
    from concurrent import futures

    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=50051)
    parser.add_argument("--max-workers", type=int, default=10)
    parser.add_argument("--ros-args", action="store_true")
    parser.add_argument("reachy_config", type=str)
    args = parser.parse_args()

    servicer = ReachyGRPCJointSDKServicer(reachy_config_path=args.reachy_config)
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=args.max_workers))

    servicer.register_all_services(server)

    server.add_insecure_port(f"[::]:{args.port}")
    server.start()

    servicer.logger.info(f"Server started on port {args.port}.")
    server.wait_for_termination()


if __name__ == "__main__":
    main()
