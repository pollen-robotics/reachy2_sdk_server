import pathlib

import grpc

# import rclpy
from .meta_rclpy import MetaRclpy
from google.protobuf.empty_pb2 import Empty
from google.protobuf.wrappers_pb2 import BoolValue
from reachy2_sdk_api.component_pb2 import ComponentId
from reachy2_sdk_api.sound_pb2 import (
    ListOfMicrophone,
    ListOfSound,
    ListOfSpeaker,
    Microphone,
    RecordingAck,
    RecordingRequest,
    SoundAck,
    SoundId,
    Speaker,
    TextRequest,
    VolumeRequest,
)
from reachy2_sdk_api.sound_pb2_grpc import add_SoundServiceServicer_to_server
from sound_play.libsoundplay import SoundClient

from ..utils import get_list_audio_files
from .audio_recorder import AudioRecorder


class ReachyGRPCAudioSDKServicer:
    def __init__(self) -> None:
        MetaRclpy.init()
        self.node = MetaRclpy.create_node("ReachyGRPCAudioSDKServicer_node")

        self.soundhandle = SoundClient(self.node, blocking=False)

        self._audiorecorder = AudioRecorder()

        self._volume: float = 1  # in [0.0,1.0]

        self.node.get_logger().info("Reachy GRPC Audio SDK Servicer initialized.")

    def register_to_server(self, server: grpc.Server):
        self.node.get_logger().info("Registering 'SoundServiceServicer' to server.")
        add_SoundServiceServicer_to_server(self, server)

    def GetAllMicrophone(self, request: Empty, context: grpc.ServicerContext) -> ListOfMicrophone:
        return ListOfMicrophone(microphone_info=[Microphone(id=ComponentId(id=1, name="microphone_1"))])

    def GetAllSpeaker(self, request: Empty, context: grpc.ServicerContext) -> ListOfSpeaker:
        return ListOfSpeaker(speaker_info=[Speaker(id=ComponentId(id=1, name="speaker_1"))])

    def StartRecording(self, request: RecordingRequest, context: grpc.ServicerContext) -> SoundAck:
        file_name = request.recording_id.id + ".ogg"
        if not pathlib.Path(file_name).parent.absolute().exists():
            self.node.get_logger().error(f"Path does not exist {file_name}")
            return RecordingAck(ack=SoundAck(success=BoolValue(value=False)))
        self.node.get_logger().info(f"Start recording {file_name}")
        self._audiorecorder.make_pipe(file_name)
        self._audiorecorder.start()
        return RecordingAck(ack=SoundAck(success=BoolValue(value=True)))

    def StopRecording(self, request: ComponentId, context: grpc.ServicerContext) -> RecordingAck:
        self.node.get_logger().info("Stop recording")
        self._audiorecorder.stop()
        return RecordingAck(ack=SoundAck(success=BoolValue(value=True)))

    def TestSpeaker(self, request: ComponentId, context: grpc.ServicerContext) -> Empty:
        self.soundhandle.playWave("say-beep.wav", volume=self._volume)
        return Empty()

    def ChangeVolume(self, request: VolumeRequest, context: grpc.ServicerContext) -> Empty:
        self.node.get_logger().info(f"Set volume to {request.volume}")
        self._volume = request.volume
        return Empty()

    def PlaySound(self, request: SoundId, context: grpc.ServicerContext) -> Empty:
        self.node.get_logger().info(f"Playing {request.sound.id}")
        # could be a wav or ogg
        self.soundhandle.playWave(request.sound.id, volume=self._volume)
        return Empty()

    def StopSound(self, request: ComponentId, context: grpc.ServicerContext) -> Empty:
        self.node.get_logger().info(f"Stop playing")
        self.soundhandle.stopAll()
        return Empty()

    def GetSoundsList(self, request: Empty, context: grpc.ServicerContext) -> ListOfSound:
        audiofiles = get_list_audio_files("/root/sounds/")
        soundsList = [SoundId(id=sound) for sound in audiofiles]
        return ListOfSound(sounds=soundsList)

    def SayText(self, request: TextRequest, context: grpc.ServicerContext) -> Empty:
        self.node.get_logger().info(f"Saying {request.text}")
        self.soundhandle.say(request.text)
        return Empty()


def main():
    import argparse
    from concurrent import futures

    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=50063)
    parser.add_argument("--max-workers", type=int, default=10)
    parser.add_argument("--ros-args", action="store_true")
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
