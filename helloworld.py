"""
需求：在终端输出文本helloworld
流程：
    1.导包  ；
    2.初始化ros2客户端；
    3.创建节点
    4.输出日志
    5.释放资源
"""
#1.导包  
import rclpy

def main():
    #2.初始化ros2客户端；
    rclpy.init()
    #3.创建节点
    node = rclpy.create_node("helloworld_node_py")
    #4.输出日志
    node.get_logger().info("hello world!(python)")
    #5.释放资源
    
    rclpy.shutdown()
    
    
if __name__ == '__main__':
    main()
    

