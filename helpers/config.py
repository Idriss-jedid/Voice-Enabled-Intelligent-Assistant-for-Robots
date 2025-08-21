"""
Configuration management for Robot Voice Pipeline.
Centralizes all environment variables and configuration settings.
"""

import os
from typing import Optional
from dotenv import load_dotenv

# Load environment variables from .env file
load_dotenv()

class Config:
    """Main configuration class for the Robot Voice Pipeline."""

    # =============================================================================
    # API Keys
    # =============================================================================
    OPENROUTER_API_KEY: str = os.getenv("OPENROUTER_API_KEY", "")

    # =============================================================================
    # Model Configuration
    # =============================================================================
    # ASR Models
    WAV2VEC2_MODEL: str = os.getenv("WAV2VEC2_MODEL", "facebook/wav2vec2-base-960h")
    VOSK_LANGUAGE: str = os.getenv("VOSK_LANGUAGE", "en-us")

    # TTS Model
    SPEECHT5_MODEL: str = os.getenv("SPEECHT5_MODEL", "microsoft/speecht5_tts")
    SPEECHT5_VOCODER: str = os.getenv("SPEECHT5_VOCODER", "microsoft/speecht5_hifigan")

    # LLM Model
    LLM_MODEL: str = os.getenv("LLM_MODEL", "mistralai/mistral-7b-instruct:free")

    # =============================================================================
    # Audio Configuration
    # =============================================================================
    SAMPLE_RATE: int = int(os.getenv("SAMPLE_RATE", "16000"))
    TTS_SPEED: float = float(os.getenv("TTS_SPEED", "1.0"))

    # =============================================================================
    # ROS2 Configuration
    # =============================================================================
    ROS_DOMAIN_ID: int = int(os.getenv("ROS_DOMAIN_ID", "0"))
    ROBOT_TOPIC: str = os.getenv("ROBOT_TOPIC", "/turtle1/cmd_vel")

    # =============================================================================
    # Output Configuration
    # =============================================================================
    OUTPUT_DIR: str = os.getenv("OUTPUT_DIR", "./")
    TTS_OUTPUT_FILE: str = os.getenv("TTS_OUTPUT_FILE", "output.wav")
    ROBOT_RESPONSE_FILE: str = os.getenv("ROBOT_RESPONSE_FILE", "robot_response.wav")

    # =============================================================================
    # Validation Methods
    # =============================================================================
    @classmethod
    def validate_required_keys(cls) -> bool:
        """Validate that all required configuration is present."""
        if not cls.OPENROUTER_API_KEY:
            raise ValueError("OPENROUTER_API_KEY is required but not set in .env file")
        return True

    @classmethod
    def get_model_config(cls) -> dict:
        """Get model configuration as dictionary."""
        return {
            "wav2vec2_model": cls.WAV2VEC2_MODEL,
            "vosk_language": cls.VOSK_LANGUAGE,
            "speecht5_model": cls.SPEECHT5_MODEL,
            "speecht5_vocoder": cls.SPEECHT5_VOCODER,
            "llm_model": cls.LLM_MODEL
        }

    @classmethod
    def get_audio_config(cls) -> dict:
        """Get audio configuration as dictionary."""
        return {
            "sample_rate": cls.SAMPLE_RATE,
            "tts_speed": cls.TTS_SPEED
        }

    @classmethod
    def get_ros2_config(cls) -> dict:
        """Get ROS2 configuration as dictionary."""
        return {
            "domain_id": cls.ROS_DOMAIN_ID,
            "robot_topic": cls.ROBOT_TOPIC
        }

    @classmethod
    def get_output_config(cls) -> dict:
        """Get output configuration as dictionary."""
        return {
            "output_dir": cls.OUTPUT_DIR,
            "tts_output_file": cls.TTS_OUTPUT_FILE,
            "robot_response_file": cls.ROBOT_RESPONSE_FILE
        }

# Create a global config instance
config = Config()

# Validate configuration on import
config.validate_required_keys()
