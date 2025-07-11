import rclpy 
from rclpy.node import Node

class TeleopNode(Node):
    def __init__ (self):
        super().__init__('teleop_ctrl_keyboard')
def main():
    rclpy.init()
    rclpy.spin(TeleopNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()