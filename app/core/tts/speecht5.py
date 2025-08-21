from transformers import SpeechT5Processor, SpeechT5ForTextToSpeech
from transformers import SpeechT5HifiGan
from datasets import load_dataset
import torch
import soundfile as sf
import os
import shutil
import librosa

class TextToSpeech:
    def __init__(self):
        """Initialize TTS models and load speaker embeddings"""
        try:
            # Initialize models
            self.processor = SpeechT5Processor.from_pretrained("microsoft/speecht5_tts")
            self.model = SpeechT5ForTextToSpeech.from_pretrained("microsoft/speecht5_tts")
            self.vocoder = SpeechT5HifiGan.from_pretrained("microsoft/speecht5_hifigan")
            
            # Load speaker embeddings with error handling
            self.speaker_embeddings = self._load_speaker_embeddings()
            
        except Exception as e:
            raise RuntimeError(f"Failed to initialize TTS models: {str(e)}")

    def _load_speaker_embeddings(self):
        """
        Load speaker embeddings with multiple fallback strategies:
        1. Try loading from a local cache (if available)
        2. Try loading default speaker embeddings from the model
        3. Fall back to random embeddings if all else fails
        """
        try:
            if hasattr(self.model, 'embeddings') and hasattr(self.model.embeddings, 'speaker_embedding'):
                return self.model.embeddings.speaker_embedding.weight[0].unsqueeze(0)
                
        except Exception as e:
            print(f"[WARN] Could not load default speaker embeddings: {e}")
        
        # Try loading from a local file if available
        try:
            import os
            local_embeddings_path = os.path.join(os.path.dirname(__file__), "default_speaker_embeddings.pt")
            if os.path.exists(local_embeddings_path):
                return torch.load(local_embeddings_path)
        except Exception as e:
            print(f"[WARN] Could not load local speaker embeddings: {e}")
        
        torch.manual_seed(42)  
        return torch.randn(1, 512, dtype=torch.float32)

    def _clear_dataset_cache(self):
        """Clear the Hugging Face datasets cache"""
        try:
            from datasets import config
            cache_dir = config.HF_DATASETS_CACHE
            dataset_cache_path = os.path.join(cache_dir, "Matthijs___cmu-arctic-xvectors")
            
            if os.path.exists(dataset_cache_path):
                shutil.rmtree(dataset_cache_path)
        except Exception as e:
            print(f"Could not clear cache: {e}")


    def synthesize(self, text: str, output_file: str = "output.wav", speed: float = 1) -> str:
        """
        Convert text to high-quality speech and save as WAV file with slower speed.
        Speed < 1.0 => slower; speed > 1.0 => faster
        """
        try:
            if not text.strip():
                raise ValueError("Input text cannot be empty")

            inputs = self.processor(text=text, return_tensors="pt")

            with torch.inference_mode():
                self.model.eval()
                self.vocoder.eval()
                speech = self.model.generate_speech(
                    inputs["input_ids"],
                    self.speaker_embeddings,
                    vocoder=self.vocoder
                )

            # Normalize audio
            speech = speech / speech.abs().max()

            # Convert to numpy for librosa
            speech_np = speech.numpy()

            # Stretch audio without changing pitch
            slowed_speech = librosa.effects.time_stretch(speech_np, rate=speed)

            # Save slowed audio
            sf.write(output_file, slowed_speech, samplerate=16000, subtype="PCM_16")
            print(f"[INFO] Audio saved to: {output_file}")

            return output_file

        except Exception as e:
            raise RuntimeError(f"Text-to-speech conversion failed: {str(e)}")

    def text_to_speech(self, text: str, output_file: str = "output.wav") -> str:
        """Alias for synthesize method for backward compatibility"""
        return self.synthesize(text, output_file)