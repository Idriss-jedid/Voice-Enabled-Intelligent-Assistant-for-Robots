from abc import ABC, abstractmethod

class BaseTextToSpeech(ABC):
    @abstractmethod
    def synthesize(self, text: str, output_path: str = "output.wav") -> str:
        """
        Generate speech audio from text and save it to output_path.
        Returns the path to the generated audio file.
        """
        pass
