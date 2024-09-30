import asyncio
import logging
import multiprocessing as mp
import sys
import threading

import grpc
import prometheus_client as pc
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
    def __init__(self, reachy_config_path: str = None, port=None) -> None:
        rclpy.init()

        # executor = rclpy.executors.MultiThreadedExecutor()
        # executor.add_node(self.bridge_node)
        # threading.Thread(target=executor.spin).start()

        # self.asyncio_loop = asyncio.get_event_loop()
        self.asyncio_loop = asyncio.new_event_loop()

        self.bridge_node = AbstractBridgeNode(reachy_config_path=reachy_config_path, asyncio_loop=self.asyncio_loop, port=port)

        self.asyncio_thread = threading.Thread(target=self.spin_asyncio)
        self.asyncio_thread.start()

        # sanity check loop running at 1Hz
        self.asyncio_loop_sanity = asyncio.new_event_loop()
        self.asyncio_thread_sanity = threading.Thread(target=self.spin_asyncio_sanity)
        self.asyncio_thread_sanity.start()

        self.logger = self.bridge_node.get_logger()

        orbita2d_servicer = Orbita2dServicer(self.bridge_node, self.logger)
        orbita3d_servicer = Orbita3dServicer(self.bridge_node, self.logger)
        arm_servicer = ArmServicer(
            self.bridge_node,
            self.logger,
            orbita2d_servicer,
            orbita3d_servicer,
        )
        hand_servicer = HandServicer(self.bridge_node, self.logger)
        head_servicer = HeadServicer(self.bridge_node, self.logger, orbita3d_servicer)
        goto_servicer = GoToServicer(self.bridge_node, self.logger)
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

    def spin_asyncio_sanity(self) -> None:
        asyncio.set_event_loop(self.asyncio_loop_sanity)
        self.asyncio_loop_sanity.run_until_complete(self.spinning_sanity(self.bridge_node))

    async def spinning_sanity(self, node):
        with node.sum_spin_sanity.time():
            await asyncio.sleep(1)

    async def spinning(self, node):
        while rclpy.ok():
            with node.sum_spin.time():
                rclpy.spin_once(node, timeout_sec=0.01)
            await asyncio.sleep(0.001)


class Interceptor(grpc.ServerInterceptor):
    summaries = {}
    general_sum = pc.Summary("sdkserver_RPC_time", f"Time spent during sdkserver ALL RPC call")

    @staticmethod
    def _wrap_rpc_behavior(handler, fn):
        """Returns a new rpc handler that wraps the given function"""
        # ref: https://github.com/mehrdada/grpc/blob/aa477becd1a7c44f8150ad24539cf6d40af24b37/examples/python/interceptors/service-latency-interceptor/service_latency_interceptor.py
        if handler is None:
            return None

        if handler.request_streaming and handler.response_streaming:
            behavior_fn = handler.stream_stream
            handler_factory = grpc.stream_stream_rpc_method_handler
        elif handler.request_streaming and not handler.response_streaming:
            behavior_fn = handler.stream_unary
            handler_factory = grpc.stream_unary_rpc_method_handler
        elif not handler.request_streaming and handler.response_streaming:
            behavior_fn = handler.unary_stream
            handler_factory = grpc.unary_stream_rpc_method_handler
        else:
            behavior_fn = handler.unary_unary
            handler_factory = grpc.unary_unary_rpc_method_handler

        return handler_factory(
            fn(behavior_fn, handler.request_streaming, handler.response_streaming),
            request_deserializer=handler.request_deserializer,
            response_serializer=handler.response_serializer,
        )

    def intercept_service(self, continuation, handler_call_details):
        # key = f"sdkserver_time__{handler_call_details.method.replace('.', '_').replace('/','_')}"
        # if key not in self.summaries:
        #     print("joint_server.grpc.Interceptor.add summary:", key)
        #     self.summaries[key] = pc.Summary(key, f"Time spent during {handler_call_details.method}")
        # with self.summaries[key].time():
        #     return continuation(handler_call_details)

        # print("call:", handler_call_details.method)
        def metrics_wrapper(behavior, request_streaming, response_streaming):
            def new_behavior(request_or_iterator, servicer_context):
                with self.general_sum.time():
                    result = behavior(request_or_iterator, servicer_context)
                    # print("call:", handler_call_details.method, "<---- end")
                return result

            return new_behavior

        return self._wrap_rpc_behavior(continuation(handler_call_details), metrics_wrapper)


_LOGGER = logging.getLogger(__name__)
handler = logging.StreamHandler(sys.stdout)
formatter = logging.Formatter("[PID %(process)d] %(message)s")
handler.setFormatter(formatter)
_LOGGER.addHandler(handler)
_LOGGER.setLevel(logging.INFO)


def main_singleprocess(_=1):
    import argparse
    from concurrent import futures

    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=5005)
    parser.add_argument("--max-workers", type=int, default=10)
    parser.add_argument("--ros-args", action="store_true")
    parser.add_argument("reachy_config", type=str)
    args = parser.parse_args()

    port = f"{args.port}{_}"
    _LOGGER.info(f"Starting grpc server at {port}")

    # options = (("grpc.so_reuseport", 1),)
    servicer = ReachyGRPCJointSDKServicer(reachy_config_path=args.reachy_config, port=port)
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=args.max_workers), interceptors=[Interceptor()])
    # server = grpc.server(futures.ThreadPoolExecutor(max_workers=args.max_workers),
    #                      options=options)
    # server = grpc.server(futures.ThreadPoolExecutor(max_workers=100))
    # server = grpc.server(futures.ProcessPoolExecutor(max_workers=4))

    servicer.register_all_services(server)

    server.add_insecure_port(f"[::]:{port}")
    _LOGGER.info("Ready to start")
    server.start()

    servicer.logger.info(f"Server started on port {port}.")
    _LOGGER.info(f"Server started on port {port}.")
    server.wait_for_termination()


def main_multiprocess():
    _LOGGER.info("Launch grpc servers...")
    workers = []
    for _ in range(1, 7):
        _LOGGER.info(f"grpc round {_}")
        # NOTE: It is imperative that the worker subprocesses be forked before
        # any gRPC servers start up. See
        # https://github.com/grpc/grpc/issues/16001 for more details.
        worker = mp.Process(target=main_singleprocess, args=(_,))
        worker.start()
        workers.append(worker)

    for worker in workers:
        worker.join()


# NOTE this code allows using a multi-process grpc server
# main = main_multiprocess
main = main_singleprocess

if __name__ == "__main__":
    main()
