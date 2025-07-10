"""
需求：编写第一个小程序，控制机器狗打招呼
流程：
  1.创建发布对象
  2.创建定时器
  3.定时器中，调用发布对象发布指令。

"""


import rclpy
from rclpy.node import Node
from unitree_api.msg import Request

class HelloWorldpy(Node):
    def __init__(self):
        super().__init__('helloworld_py')
        # 1.创建发布对象
        self.pub = self.create_publisher(Request,"/api/sport/request",10)
        # 2.创建定时器
        self.timer = self.create_timer(1.0, self.on_timer)
    def on_timer(self):
        request = Request()
        request.header.identity.api_id = 1016
        self.pub.publish(request)
            

def main():
    rclpy.init()
    rclpy.spin(HelloWorldpy())
    rclpy.shutdown()

if __name__ == '__main__':
    main()