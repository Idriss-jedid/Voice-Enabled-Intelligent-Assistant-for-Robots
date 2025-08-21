import rclpy
from app.core.orchestrator import PipelineOrchestrator
from app.ros2_client.turtle_controller import TurtleMovementPublisher
from app.core.tts.speecht5 import TextToSpeech
import torchaudio
import sounddevice as sd
import numpy as np
import argparse
import warnings
import logging
import os
import sys
from io import StringIO

# Suppress model loading warnings and logs at startup
warnings.filterwarnings("ignore")


_orchestrator = None
_tts = None

def initialize_models():
    """Initialize models once at startup"""
    global _orchestrator, _tts

    if _orchestrator is None:

        old_stdout = sys.stdout
        old_stderr = sys.stderr
        sys.stdout = StringIO()
        sys.stderr = StringIO()

        try:
            _orchestrator = PipelineOrchestrator()
            _tts = TextToSpeech()
        finally:
            sys.stdout = old_stdout
            sys.stderr = old_stderr


    return _orchestrator, _tts

def record_audio(duration=5, fs=16000):
    """Record audio from microphone"""
    print(f"🎤 Recording audio for {duration} seconds...")
    recording = sd.rec(int(duration * fs), samplerate=fs, channels=1, dtype='float32')
    sd.wait() 
    return torch.from_numpy(recording).float(), fs

def main(audio, sr, input_source="text"):
    orchestrator, _ = initialize_models()

    robot_response = orchestrator.run_pipeline(audio, sr)

  
   
def get_user_input(prompt):
    """Safely get user input with proper encoding handling"""
    while True:
        try:
            return input(prompt)
        except UnicodeDecodeError:
            continue

def process_text_input():
    """Process text input by converting it to speech and then processing through pipeline"""
    text = get_user_input("Enter the command for the robot (will be synthesized to audio, type 'exit' to quit): ")
    if text.lower() == "exit":
        return False

    try:

        old_stdout = sys.stdout
        sys.stdout = StringIO()

        try:
            tts.synthesize(text, "tts_input.wav")
        finally:
            sys.stdout = old_stdout

        audio, sr = torchaudio.load("tts_input.wav")
        main(audio, sr, input_source="text")
    except Exception as e:
        print(f"❌ Error processing command: {e}")

    return True

def process_mic_input():
    """Process microphone input"""
    print("1. Record a new command")
    print("2. Exit")

    choice = input("Select an option (1 or 2): ")

    if choice == "1":
        duration = 5
        try:
            duration = int(input(f"Enter recording duration in seconds (default {duration}): ") or duration)
        except ValueError:
            print("Using default duration")

        audio, sr = record_audio(duration)
        main(audio, sr, input_source="microphone")
    elif choice == "2":
        return False
    else:
        print("Invalid option")

    return True

def process_file_input():
    """Process WAV file input"""
    file_path = input("Enter the path to the WAV file (or 'exit' to quit): ")

    if file_path.lower() == "exit":
        return False

    if not os.path.exists(file_path):
        return True

    try:
        audio, sr = torchaudio.load(file_path)
        main(audio, sr, input_source="file")
    except Exception as e:
        print(f"❌ Error processing file: {e}")

    return True

def main_menu():
    """Main menu for selecting input method"""
    while True:

        print("1. Microphone Input")
        print("2. WAV File Input")
        print("3. Exit")

        choice = input("Select an option (1-3): ")


        if choice == "1":
            if not process_mic_input():
                break
        elif choice == "2":
            if not process_file_input():
                break
        elif choice == "3":
            print("Exiting...")
            break
        else:
            print("Invalid option. Please try again.")

if __name__ == '__main__':
    _, tts = initialize_models()

    try:
        main_menu()
    except KeyboardInterrupt:
        print("👋 Exiting program...")
    except Exception as e:
        print(f"❌ Unexpected error: {e}")
