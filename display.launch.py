from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command,LaunchConfiguration
import os
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    #获取功能包路径
    go2_description_pkg = get_package_share_directory("go2_description")
    use_joint_state_publisher = DeclareLaunchArgument(
        name = "use_joint_state_publisher",
        default_value="true"
    )
    model =DeclareLaunchArgument(
        name = "urdf_path",
        default_value = os.path.join(go2_description_pkg,"urdf","go2_description.urdf")
    )
    #使用xacro读取urdf文件里面的内容
    # robot_desc = ParameterValue(Command(["xacro ",os.path.join(go2_description_pkg,"urdf","go2_description.urdf")]))
    robot_desc = ParameterValue(Command(["xacro ",LaunchConfiguration("urdf_path")]))
    #robot_state_publisher ----加载机器人urdf文件
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description":robot_desc}]
    )
    #joint_state_publisher ----发布关节状态
    #以后更合理的方式是由程序动态获取关节信息并发布
    #这个节点的启动应该是有附加条件的
    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        condition = IfCondition(LaunchConfiguration("use_joint_state_publisher"))

    )
    return LaunchDescription([
        model,
        use_joint_state_publisher,
        robot_state_publisher,
        joint_state_publisher

    ])