from abc import ABC, abstractmethod


class ASRBase(ABC):
    @abstractmethod
    def transcribe(self, audio, sr: int) -> str:
        pass
