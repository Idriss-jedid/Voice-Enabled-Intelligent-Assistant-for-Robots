# 🤖 Robot Voice Pipeline

<div align="center">
  <img src="https://img.shields.io/badge/Python-3.8%2B-blue" alt="Python Version">
  <img src="https://img.shields.io/badge/ROS2-Humble%2B-green" alt="ROS2 Version">
  <img src="https://img.shields.io/badge/License-MIT-yellow.svg" alt="License">
  <img src="https://img.shields.io/badge/Status-Active-brightgreen" alt="Project Status">
  <img src="https://img.shields.io/badge/Input-Microphone%20%7C%20File%20%7C%20Text-blueviolet" alt="Input Methods">
</div>

<p align="center">
  <a href="#about">About</a> •
  <a href="#features">Features</a> •
  <a href="#installation">Installation</a> •
  <a href="#usage">Usage</a> •
  <a href="#architecture">Architecture</a> •
  <a href="#examples">Examples</a>
</p>

## About

Robot Voice Pipeline is a comprehensive speech-to-robot control system that enables natural language interaction with ROS2 robots. The system processes voice commands through a multi-stage pipeline involving speech recognition, language understanding, and robot movement control.

## ✨ Features

- **Multiple Input Methods**:
  - 🎤 Real-time microphone input
  - 📁 WAV file processing
  - ⌨️ Direct text input
- **Advanced ASR**: Combines Wav2Vec2 and Vosk for robust speech recognition
- **LLM Integration**: Uses OpenRouter API for intelligent command interpretation  
- **Text-to-Speech**: Microsoft's SpeechT5 for natural speech synthesis
- **ROS2 Control**: Direct integration with turtle simulator (extensible to other robots)
- **Error Correction**: Automatic transcription correction using LLM
- **Interactive CLI**: User-friendly command-line interface
- **Real-time Processing**: Optimized for interactive robot control

## Architecture

```
┌─────────────┐   ┌─────────────┐   ┌─────────────┐   ┌─────────────┐
│             │   │             │   │             │   │             │
│   Input     │──▶│    ASR      │──▶│     LLM     │──▶│   Robot     │
│  (Mic/File/ │   │  (Speech    │   │  (Command   │   │  Control    │
│    Text)    │   │ Recognition)│   │  Processing)│   │             │
│             │   │             │   │             │   │             │
└─────────────┘   └─────────────┘   └─────────────┘   └─────────────┘
                                    │
                                    ▼
                             ┌─────────────┐
                             │             │
                             │    TTS      │
                             │  (Speech    │
                             │  Response)  │
                             │             │
                             └─────────────┘
```

## 🚀 Installation

### Prerequisites

- Python 3.8+
- ROS2 (Humble recommended)
- PyTorch with audio support
- [OpenRouter API key](https://openrouter.ai/)
- Sound device with microphone support (for live input)

### 🛠️ Setup

1. **Clone the repository**
   ```bash
   git clone https://github.com/Idriss-jedid/robot_voice_pipeline.git
   cd robot_voice_pipeline
   ```

2. **Create and activate virtual environment** (recommended)
   ```bash
   python -m venv venv
   source venv/bin/activate  # On Windows: .\venv\Scripts\activate
   ```

3. **Install dependencies**
   ```bash
   pip install -r requirements.txt
   ```

4. **Configure environment**
   ```bash
   cp .env.example .env
   # Edit .env with your OpenRouter API key and preferences
   ```

5. **Setup ROS2** (if not already installed)
   ```bash
   # Follow ROS2 installation guide for your OS
   # For Ubuntu 22.04:
   # sudo apt update && sudo apt install -y curl gnupg lsb-release
   # sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
   # echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
   # sudo apt update
   # sudo apt install ros-humble-desktop
   sudo apt install -y ros-humble-desktop
   ```

3. **Set up environment**:
   ```bash
   source /opt/ros/humble/setup.bash
   ```

4. **Add sourcing to your .bashrc** (optional but recommended):
   ```bash
   echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
   source ~/.bashrc
   ```

## Usage

### Basic Usage

1. **Run the main pipeline**
   ```bash
   python main_pipeline_with_multiple_inputs.py
   ```

2. **Select input method**:
   - "Move forward"
   - "Turn left" 
   - "Stop"
   - "What's your name?"

3. **The system will:**
   - Convert text to speech (TTS)
   - Process the audio through ASR
   - Interpret commands via LLM
   - Control the robot and respond verbally

### Example Commands

- **Movement**: "Go forward","Turn left", "Turn right", "Move backward", "Stop"
- **Questions**: "Hello robot", "What can you do?", "How are you?"
- **Complex**: "Move forward then turn left"

## Configuration

The project uses a centralized configuration system via `helpers/config.py` that loads settings from environment variables.

### Environment Variables (.env)

```bash
# API Keys (Required)
OPENROUTER_API_KEY=your_key_here

# Model Configuration  
WAV2VEC2_MODEL=facebook/wav2vec2-base-960h
VOSK_LANGUAGE=en-us
SPEECHT5_MODEL=microsoft/speecht5_tts
SPEECHT5_VOCODER=microsoft/speecht5_hifigan
LLM_MODEL=mistralai/mistral-7b-instruct:free

# Audio Settings
SAMPLE_RATE=16000
TTS_SPEED=1.0

# ROS2 Settings
ROS_DOMAIN_ID=0
ROBOT_TOPIC=/turtle1/cmd_vel

# Output Settings
OUTPUT_DIR=./
TTS_OUTPUT_FILE=output.wav
ROBOT_RESPONSE_FILE=robot_response.wav
```

### Using Configuration in Code

```python
from helpers.config import config

# Access configuration values
api_key = config.OPENROUTER_API_KEY
model_name = config.WAV2VEC2_MODEL
sample_rate = config.SAMPLE_RATE

# Get configuration groups
model_config = config.get_model_config()
audio_config = config.get_audio_config()
ros2_config = config.get_ros2_config()
```

### Extending the System

#### Adding New ASR Models

```python
# app/core/asr/my_asr.py
from app.core.asr.base import ASRBase

class MyASR(ASRBase):
    def transcribe(self, audio, sr: int) -> str:
        # Your implementation
        return transcribed_text
```

#### Adding New Robot Controllers

```python
# app/ros2_client/my_robot.py
from app.ros2_client.turtle_controller import TurtleMovementPublisher

class MyRobotController(TurtleMovementPublisher):
    def move_robot(self, movements):
        # Your robot-specific implementation
        pass
```

## Project Structure

```
robot_voice_pipeline/
├── app/
│   ├── core/
│   │   ├── asr/          # Speech recognition modules
│   │   ├── llm/          # Language model integration
│   │   ├── tts/          # Text-to-speech modules
│   │   └── orchestrator.py
│   └── ros2_client/      # ROS2 integration
├── main_pipeline_with_ros.py
├── requirements.txt
├── .env.example
└── README.md
```

## Dependencies

- **torch & torchaudio**: PyTorch for ML models
- **transformers**: Hugging Face models (Wav2Vec2, SpeechT5)
- **vosk**: Offline speech recognition
- **datasets**: Hugging Face datasets
- **soundfile & librosa**: Audio processing
- **pydantic & langchain**: Data validation and LLM integration
- **rclpy**: ROS2 Python client
- **requests**: HTTP client for OpenRouter API

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Acknowledgments

- [Wav2Vec2](https://github.com/pytorch/fairseq/tree/main/examples/wav2vec2) for speech recognition
- [Vosk](https://alphacephei.com/vosk/) for offline ASR
- [SpeechT5](https://github.com/microsoft/speecht5) for text-to-speech
- [OpenRouter](https://openrouter.ai/) for LLM access
- [ROS2](https://www.ros.org/) for robot control

## Advanced Usage

### Using Pre-generated Speech Files

For testing and development, you can generate speech files directly and use them in the pipeline:

