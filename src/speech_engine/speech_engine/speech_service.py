import rclpy
from rclpy.node import Node
from custom_services.srv import Speech
import pyttsx3 

class TextToSpeechService(Node):
    def __init__(self):
        super().__init__('speech_service')

        #  speech engine setup
        self.tts_engine = pyttsx3.init()
        self.tts_engine.setProperty('rate', 150) #speech speed
        self.tts_engine.setProperty('volume', 0.9)  #volume

        #  ROS2 service setup
        self.service = self.create_service(
            Speech,  
            'speak_text',  
            self.handle_speak_text
        )

        self.get_logger().info('Speech Service ready')

    def handle_speak_text(self, request, response):
        text_request = request.text 
        self.get_logger().info(f'Received request: {text_request}')

        try:
            self.tts_engine.say(text_request)
            self.tts_engine.runAndWait()
            response.success = True
        except Exception as e:
            response.success = False

        return response


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
