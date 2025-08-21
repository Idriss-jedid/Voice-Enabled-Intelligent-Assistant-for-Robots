from app.core.asr.base import ASRBase
from transformers import Wav2Vec2ForCTC, Wav2Vec2Processor
import torch
from helpers.config import config


class Wav2Vec2ASR(ASRBase):
    def __init__(self, model_id: str = None):
        """
        Initialize Wav2Vec2 ASR model.

        Args:
            model_id: Model identifier (defaults to config.WAV2VEC2_MODEL)
        """
        if model_id is None:
            model_id = config.WAV2VEC2_MODEL

        self.processor = Wav2Vec2Processor.from_pretrained(model_id)
        self.model = Wav2Vec2ForCTC.from_pretrained(model_id).eval()

    def transcribe(self, audio, sr: int) -> str:
        """
        Transcribe audio using Wav2Vec2 model.

        Args:
            audio: Audio tensor
            sr: Sample rate

        Returns:
            Transcribed text
        """
        input_values = self.processor(audio.squeeze().numpy(), return_tensors="pt", sampling_rate=sr).input_values
        with torch.no_grad():
            logits = self.model(input_values).logits
        pred_ids = torch.argmax(logits, dim=-1)
        transcription = self.processor.decode(pred_ids[0])
        return transcription.strip()
