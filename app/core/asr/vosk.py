from app.core.asr.base import ASRBase
from vosk import Model as VoskModel, KaldiRecognizer
import torchaudio
import json
import tempfile
import wave
from helpers.config import config

class VoskASR(ASRBase):
    def __init__(self, vosk_lang: str = None):
        """
        Initialize Vosk ASR model.

        Args:
            vosk_lang: Language code (defaults to config.VOSK_LANGUAGE)
        """
        if vosk_lang is None:
            vosk_lang = config.VOSK_LANGUAGE
        self.model = VoskModel(lang=vosk_lang)

    def transcribe(self, audio, sr: int) -> str:
        try:
            # Ensure audio is mono and in the right format
            if audio.dim() > 1:
                audio = audio.mean(dim=0, keepdim=True)
            
            # Convert to 16kHz if needed
            if sr != 16000:
                resampler = torchaudio.transforms.Resample(sr, 16000)
                audio = resampler(audio)
                sr = 16000
            
            # Use context manager for proper cleanup
            with tempfile.NamedTemporaryFile(suffix=".wav") as tmp:
                torchaudio.save(tmp.name, audio, sr)
                
                recognizer = KaldiRecognizer(self.model, sr)
                
                with wave.open(tmp.name, 'rb') as wf:
                    results = []
                    while True:
                        data = wf.readframes(4000)
                        if len(data) == 0:
                            break
                        
                        if recognizer.AcceptWaveform(data):
                            result = json.loads(recognizer.Result())
                            if result.get("text"):
                                results.append(result["text"])
                    
                    final_result = json.loads(recognizer.FinalResult())
                    if final_result.get("text"):
                        results.append(final_result["text"])
                    
                    return " ".join(results).strip()
                    
        except Exception as e:
            print(f"⚠️ Vosk transcription error: {e}")
            return ""