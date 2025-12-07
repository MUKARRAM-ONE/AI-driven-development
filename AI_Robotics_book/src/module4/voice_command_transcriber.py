import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pyaudio
import wave
import openai
import os
import io
import logging

class VoiceCommandTranscriber(Node):
    def __init__(self):
        super().__init__('voice_command_transcriber')
        self.publisher_ = self.create_publisher(String, '/voice_command/text', 10)
        self.get_logger().info('Voice Command Transcriber node started.')

        # Initialize OpenAI API client (replace with your API key or set as env var)
        openai.api_key = os.getenv("OPENAI_API_KEY", "YOUR_OPENAI_API_KEY") # Replace YOUR_OPENAI_API_KEY if not using env var

        # Audio recording parameters
        self.FORMAT = pyaudio.paInt16
        self.CHANNELS = 1
        self.RATE = 16000 # Sample rate for Whisper
        self.CHUNK = 1024 # Buffer size
        self.RECORD_SECONDS = 5 # Record for 5 seconds per command

        self.audio = pyaudio.PyAudio()
        self.stream = None # Will be opened when recording starts

        self.get_logger().info("Ready to record voice commands. Set OPENAI_API_KEY environment variable.")
        self.get_logger().info("Call record_and_transcribe() method to start recording.")

    def record_and_transcribe(self):
        self.get_logger().info("Recording...")
        frames = []
        try:
            self.stream = self.audio.open(format=self.FORMAT,
                                          channels=self.CHANNELS,
                                          rate=self.RATE,
                                          input=True,
                                          frames_per_buffer=self.CHUNK)
            
            for _ in range(0, int(self.RATE / self.CHUNK * self.RECORD_SECONDS)):
                data = self.stream.read(self.CHUNK, exception_on_overflow=False)
                frames.append(data)
            
            self.get_logger().info("Finished recording. Transcribing...")
            
            self.stream.stop_stream()
            self.stream.close()

            audio_file = io.BytesIO()
            wf = wave.open(audio_file, 'wb')
            wf.setnchannels(self.CHANNELS)
            wf.setsampwidth(self.audio.get_sample_size(self.FORMAT))
            wf.setframerate(self.RATE)
            wf.writeframes(b''.join(frames))
            wf.close()
            
            audio_file.name = "voice_command.wav"
            audio_file.seek(0)

            if openai.api_key == "YOUR_OPENAI_API_KEY" or not openai.api_key:
                 self.get_logger().error("OpenAI API key not set. Please set OPENAI_API_KEY environment variable.")
                 return
            
            response = openai.Audio.transcribe("whisper-1", audio_file)
            transcribed_text = response['text']
            self.get_logger().info(f"Transcribed: '{transcribed_text}'")

            msg = String()
            msg.data = transcribed_text
            self.publisher_.publish(msg)
            self.get_logger().info(f"Published voice command: '{transcribed_text}'")
        
        except Exception as e:
            self.get_logger().error(f"Error during recording or transcription: {e}")
        finally:
            if self.stream is not None:
                if self.stream.is_active():
                    self.stream.stop_stream()
                self.stream.close()

def main(args=None):
    logging.basicConfig(level=logging.INFO)
    rclpy.init(args=args)
    transcriber_node = VoiceCommandTranscriber()
    # To run, manually call the record_and_transcribe method
    # transcriber_node.record_and_transcribe() 
    rclpy.spin(transcriber_node) # Keep node alive to be able to call method
    transcriber_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    # For demonstration, you might want to call record_and_transcribe here
    # However, in a real ROS 2 setup, this would be triggered by an external event or service call.
    # To run this script as a standalone (for testing the transcription),
    # uncomment the transcriber_node.record_and_transcribe() line in main() and add rclpy.shutdown() after it.
    main()
