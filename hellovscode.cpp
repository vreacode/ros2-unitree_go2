/*
需求：在终端输出文本helloworld
流程：
2.初始化ros2客户端；
3.创建节点指针
4.输出日志
5.释放资源


*/

// 1.包含头文件； 
#include "rclcpp/rclcpp.hpp"


int main(int argc, char const *argv[])
{
// 2.初始化ros2客户端；
rclcpp::init(argc,argv);
// 3.创建节点指针
auto node = rclcpp::Node::make_shared("hellovscode_node_cpp");
// 4.输出日志
RCLCPP_INFO(node->get_logger(),"hello vscode.....");
// 5.释放资源
rclcpp::shutdown();
  return 0;
}
