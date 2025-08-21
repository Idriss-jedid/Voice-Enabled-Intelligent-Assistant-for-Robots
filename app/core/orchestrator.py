from app.core.asr.wav2vec2 import Wav2Vec2ASR
from app.core.asr.vosk import VoskASR
from app.core.llm.corrections import correct_text
from app.core.llm.router import call_openrouter
from app.core.tts.speecht5 import TextToSpeech
from app.ros2_client.turtle_controller import TurtleMovementPublisher
import torchaudio

class PipelineOrchestrator:
    def __init__(self):
        # Initialize ASR modules

        self.wav2vec = Wav2Vec2ASR()
        self.vosk = VoskASR()

        # Initialize TTS
        self.tts = TextToSpeech()

    def load_audio(self, audio_path):
        audio, sr = torchaudio.load(audio_path)
        return audio, sr

    def transcribe_audio(self, audio, sr):
        vosk_text = self.vosk.transcribe(audio, sr)
        wav2vec_text = self.wav2vec.transcribe(audio, sr)

        return wav2vec_text, vosk_text

    def run_pipeline(self, audio, sr):
        try:
            wav2vec_text, vosk_text = self.transcribe_audio(audio, sr)
            combined = f"{wav2vec_text} / {vosk_text}"
            corrected_text = correct_text(combined)

            # LLM chat (robot response) using robust parser
            from app.core.llm.robot_llm_parser import chat_with_robot
            robot_response = chat_with_robot(corrected_text)

            speech = robot_response.speech
            movements = robot_response.movements

            # Run TTS
            output_tts_path = self.tts.synthesize(speech)

            # ROS2 Movement
            import rclpy
            rclpy.init()
            try:
                node = TurtleMovementPublisher()
                node.move_robot(movements)
                node.destroy_node()
            finally:
                rclpy.shutdown()

            # Return the structured response object for compatibility
            return robot_response
        except Exception as e:
            raise
