from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    start_agent = LaunchConfiguration('start_agent')
    agent_port = LaunchConfiguration('agent_port')

    start_px4 = LaunchConfiguration('start_px4')
    px4_cmd = LaunchConfiguration('px4_cmd')

    start_bridge = LaunchConfiguration('start_bridge')
    bridge_cmd = LaunchConfiguration('bridge_cmd')

    start_tf = LaunchConfiguration('start_tf')
    start_detector = LaunchConfiguration('start_detector')

    bridge_delay = LaunchConfiguration('bridge_delay')
    tf_delay = LaunchConfiguration('tf_delay')
    detector_delay = LaunchConfiguration('detector_delay')

    # 1) XRCE Agent
    agent = ExecuteProcess(
        cmd=['MicroXRCEAgent', 'udp4', '-p', agent_port],
        name='microxrce_agent',
        output='screen',
        condition=IfCondition(start_agent)
    )

    # 2) PX4 SITL + Gazebo
    px4 = ExecuteProcess(
        cmd=['bash', '-lc', px4_cmd],
        name='px4_sitl',
        output='screen',
        condition=IfCondition(start_px4)
    )

    # 3) ros_gz_bridge
    bridge = TimerAction(
        period=bridge_delay,
        actions=[
            ExecuteProcess(
                cmd=['bash', '-lc', bridge_cmd],
                name='px4_gazebo_bridge',
                output='screen',
            )
        ],
        condition=IfCondition(start_bridge)
    )

    # 4) Static TF
    static_tf = TimerAction(
        period=tf_delay,
        actions=[
            Node(
                package='px4_ros_com',
                executable='fix_tf_static',
                name='fix_tf_static',
                output='screen',
                emulate_tty=True,
            )
        ],
        condition=IfCondition(start_tf)
    )

    # 5) Detector Action Server
    detector = TimerAction(
        period=detector_delay,
        actions=[
            Node(
                package='px4_ros_com',
                executable='offboard_control_detect_srv',
                name='color_detection_service',
                output='screen',
                emulate_tty=True,
            )
        ],
        condition=IfCondition(start_detector)
    )

    return LaunchDescription([
        DeclareLaunchArgument('start_agent', default_value='true'),
        DeclareLaunchArgument('agent_port', default_value='8888'),

        DeclareLaunchArgument('start_px4', default_value='true'),
        DeclareLaunchArgument(
            'px4_cmd',
            default_value='cd ~/PX4-Autopilot && make px4_sitl gz_x500_mono_cam'
        ),

        DeclareLaunchArgument('start_bridge', default_value='true'),
        DeclareLaunchArgument(
            'bridge_cmd',
            default_value='source ~/ws/install/setup.bash && ros2 run ros_gz_bridge parameter_bridge /camera@sensor_msgs/msg/Image@gz.msgs.Image'
        ),

        DeclareLaunchArgument('start_tf', default_value='true'),
        DeclareLaunchArgument('start_detector', default_value='true'),

        DeclareLaunchArgument('bridge_delay', default_value='5.0'),
        DeclareLaunchArgument('tf_delay', default_value='8.0'),
        DeclareLaunchArgument('detector_delay', default_value='10.0'),

        agent,
        px4,
        bridge,
        static_tf,
        detector,
    ])
