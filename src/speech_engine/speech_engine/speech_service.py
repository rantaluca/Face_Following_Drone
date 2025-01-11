import rclpy
from rclpy.node import Node
from custom_services.srv import Speech
import pyttsx3 
#from TTS.api import TTS
#import os
#from espeakng import ESpeakNG
import asyncio
import edge_tts
import os
class TextToSpeechService(Node):
    def __init__(self):
        super().__init__('speech_service')

        #  speech engine setup
        # self.tts_engine = pyttsx3.init()
        # self.tts_engine.setProperty('rate', 150) #speech speed
        # self.tts_engine.setProperty('volume', 0.9)  #volume


        # self.tts_engine = TTS(model_name="tts_models/en/ljspeech/tacotron2-DDC", gpu=False)

        # self.tts_engine = ESpeakNG()
        # self.tts_engine.voice = 'en'  
        # self.tts_engine.speed = 150   
        # self.tts_engine.volume = 100  

        #       
        self.voice = "en-US-AndrewNeural"
        self.output_file = "/tmp/speech_output.mp3"
        
        #  ROS2 service setup
        self.service = self.create_service(
            Speech,  
            'speak_text',  
            self.handle_speak_text
        )

        self.get_logger().info('Speech Service ready')

    
    async def generate_speech(self, text: str):
        try:
            communicate = edge_tts.Communicate(text, self.voice)
            await communicate.save(self.output_file)
            self.get_logger().info(f"Audio generated at {self.output_file}")

            os.system(f"ffplay -nodisp -autoexit {self.output_file}")

            return True
        except Exception as e:
            self.get_logger().error(f"Error generating speech: {e}")
            return False
        
    def handle_speak_text(self, request, response):
        text_request = request.text
        self.get_logger().info(f"Received request: {text_request}")

        try:
            # Run the asynchronous TTS generation in an event loop
            loop = asyncio.get_event_loop()
            if loop.is_running():
                self.get_logger().error("Event loop already running. Cannot generate speech asynchronously.")
                response.success = False
            else:
                success = loop.run_until_complete(self.generate_speech(text_request))
                response.success = success

        except Exception as e:
            self.get_logger().error(f"Error handling request: {e}")
            response.success = False

        return response
    # def handle_speak_text(self, request, response):
    #     text_request = request.text 
    #     self.get_logger().info(f'Received request: {text_request}')

    #     try:
    #         self.tts_engine.say(text_request)
    #         self.tts_engine.runAndWait()

    #         # output_file = "/tmp/speech_output.wav"
    #         # self.tts_engine.tts_to_file(text=text_request, file_path=output_file)

    #         # # Play the generated audio file (optional: integrate with a player like `ffplay` or `aplay`)
    #         # self.get_logger().info(f"Audio generated at {output_file}")
    #         # os.system(f"ffplay -nodisp -autoexit {output_file}")

    #         #self.tts_engine.say(text_request)

    #         response.success = True
    #     except Exception as e:
    #         response.success = False

    #     return response


def main(args=None):
    rclpy.init(args=args)
    node = TextToSpeechService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down speech service')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
