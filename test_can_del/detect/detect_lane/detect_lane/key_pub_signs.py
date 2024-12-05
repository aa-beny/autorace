import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from pynput import keyboard

class KeyboardPublisher(Node):
    def __init__(self):
        super().__init__('key_signs_publisher')
        self.publisher = self.create_publisher(String, '/detect/signs', 10)
        self.publisher_traffic_light_topic = self.create_publisher(String, '/detect/traffic_light', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.key_to_publish = '0'

    def timer_callback(self):
        if self.key_to_publish is not None:
            msg = String()
            msg.data = f'Key {self.key_to_publish} pressed'

            if self.key_to_publish == '1':
                msg.data = "Intersection_sign"
            elif self.key_to_publish == '2':
                msg.data = "Left_sign"
            elif self.key_to_publish == '3':
                msg.data = "Right_sign"
            elif self.key_to_publish == '4':
                msg.data = "Stop_sign"
            elif self.key_to_publish == '5':
                msg.data = "Park_sign"
            elif self.key_to_publish == '6':
                msg.data = "Stop_Bar_sign"
            elif self.key_to_publish == '7':
                msg.data = "Tunnel_sign"
            elif self.key_to_publish == '8':
                msg.data = "Obstacle_sign"
            elif self.key_to_publish == '9':
                tr_msg = String()
                tr_msg.data = "GREEN"
                self.publisher_traffic_light_topic.publish(tr_msg)
            else:
                msg.data = ""

            self.publisher.publish(msg)

    def on_press(self, key):
        try:
            key_char = key.char
            if key_char in ['1', '2', '3', '4', '5', '6', '7', '8', '9']:
                self.key_to_publish = key_char
            else:
                self.key_to_publish = '0'
        except AttributeError:
            # Ignore non-character keys
            pass

def main(args=None):
    rclpy.init(args=args)
    keyboard_publisher = KeyboardPublisher()
    keyboard_listener = keyboard.Listener(on_press=keyboard_publisher.on_press)
    keyboard_listener.start()
    rclpy.spin(keyboard_publisher)
    keyboard_listener.stop()
    keyboard_listener.join()
    keyboard_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
