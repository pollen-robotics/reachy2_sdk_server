import grpc
import rclpy
import threading


from ..abstract_bridge_node import AbstractBridgeNode
from .arm import ArmServicer
from .orbita2d import Orbita2dServicer


class ReachyGRPCJointSDKServicer:
    def __init__(self, reachy_config_path: str = None) -> None:
        rclpy.init()
        self.bridge_node = AbstractBridgeNode(reachy_config_path=reachy_config_path)

        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(self.bridge_node)
        threading.Thread(target=executor.spin).start()
        # threading.Thread(target=lambda: rclpy.spin(self.bridge_node)).start()

        self.logger = self.bridge_node.get_logger()
        self.services = [
            ArmServicer(self.bridge_node, self.logger),
            Orbita2dServicer(self.bridge_node, self.logger),
        ]

        self.logger.info("Reachy GRPC Joint SDK Servicer initialized.")

    def register_all_services(self, server: grpc.Server) -> None:
        for serv in self.services:
            serv.register_to_server(server)


def main():
    import argparse
    from concurrent import futures

    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=50051)
    parser.add_argument("--max-workers", type=int, default=10)
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
