import time

import grpc
import rclpy
from sound_play.libsoundplay import SoundClient

from ..utils import get_list_audio_files

from google.protobuf.empty_pb2 import Empty
from google.protobuf.wrappers_pb2 import BoolValue

from reachy_sdk_api_v2.sound_pb2 import (
    ListOfMicrophone,
    Microphone,
    ListOfSpeaker,
    Speaker,
    RecordingRequest,
    RecordingAck,
    VolumeRequest,
    SoundId,
    SoundAck,
    ListOfSound,
)
from reachy_sdk_api_v2.sound_pb2_grpc import (
    add_SoundServiceServicer_to_server,
)
from reachy_sdk_api_v2.component_pb2 import ComponentId


# ToDo : move this code to actual functions called by grpc
class ReachyGRPCAudioSDKServicer:
    def __init__(self) -> None:
        rclpy.init()
        # Dummy node
        self.node = rclpy.create_node("soundclient_example")
        # note: non blocking mode for gprc?
        self.soundhandle = SoundClient(self.node, blocking=True)

        self.node.get_logger().info("Reachy GRPC Audio SDK Servicer initialized.")

    def register_to_server(self, server: grpc.Server):
        self.node.get_logger().info("Registering 'SoundServiceServicer' to server.")
        add_SoundServiceServicer_to_server(self, server)

    def test(self) -> None:
        # look into its default sounds folder if there is no path
        # https://github.com/ros-drivers/audio_common/tree/ros2/sound_play/sounds
        self.node.get_logger().info("Playing say-beep at full volume.")
        self.soundhandle.playWave("say-beep.wav")

        self.node.get_logger().info("Playing say-beep at volume 0.3.")
        self.soundhandle.playWave("say-beep.wav", volume=0.3)

    def say_text(self, text: str) -> None:
        self.node.get_logger().info(f"Say {text}")
        self.soundhandle.say(text)

    def play_sound(self, file_name: str) -> None:
        self.node.get_logger().info(f"Playing {file_name}")
        # could be a wav or ogg
        self.soundhandle.playWave(file_name)

    def stop(self) -> None:
        # note: not sure it is working. Need to double check ROS package
        self.node.get_logger().info(f"Stop playing")
        self.soundhandle.stopAll()

    def start_capture(self, filename: str) -> None:
        # Todo check filename (has to be mp3, in the sound folder)
        self.node.get_logger().info(f"Start recording {filename}")
        # Todo: connect to ROS node

    def stop_capture(self) -> None:
        self.node.get_logger().info("Stop recording")
        # Todo : connect to ROS node

    def GetAllMicrophone(self, request: Empty, context: grpc.ServicerContext) -> ListOfMicrophone:
        return ListOfMicrophone(microphone_info=[Microphone(id=ComponentId(id=1, name="microphone_1"))])

    def GetAllSpeaker(self, request: Empty, context: grpc.ServicerContext) -> ListOfSpeaker:
        return ListOfSpeaker(speaker_info=[Speaker(id=ComponentId(id=1, name="speaker_1"))])

    def StartRecording(
        self, request: RecordingRequest, context: grpc.ServicerContext
    ) -> Empty:
        file_name = request.recording_id.id + "mp3"
        self.start_capture(file_name)
        return Empty()

    def StopRecording(
        self, request: ComponentId, context: grpc.ServicerContext
    ) -> RecordingAck:
        self.stop_capture()
        return RecordingAck(ack=SoundAck(success=BoolValue(value=True)))

    def TestSpeaker(
        self, request: ComponentId, context: grpc.ServicerContext
    ) -> Empty:
        self.test()
        return Empty()

    def ChangeVolume(
        self, request: VolumeRequest, context: grpc.ServicerContext
    ) -> Empty:
        # TODO
        return Empty()

    def PlaySound(self, request: SoundId, context: grpc.ServicerContext) -> Empty:
        self.play_sound(request.id)
        return Empty()

    def StopSound(self, request: ComponentId, context: grpc.ServicerContext) -> Empty:
        return Empty()

    def GetSoundsList(self, request: Empty, context: grpc.ServicerContext) -> ListOfSound:
        audiofiles = get_list_audio_files("/root/sounds/")
        soundsList = [SoundId(id=sound) for sound in audiofiles]
        return ListOfSound(sounds=soundsList)


def main():
    import argparse
    from concurrent import futures

    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=50063)
    parser.add_argument("--max-workers", type=int, default=10)
    args = parser.parse_args()

    servicer = ReachyGRPCAudioSDKServicer()
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=args.max_workers))

    servicer.register_to_server(server)

    server.add_insecure_port(f"[::]:{args.port}")
    server.start()

    servicer.node.get_logger().info(f"Server started on port {args.port}.")
    server.wait_for_termination()


if __name__ == "__main__":
    main()
