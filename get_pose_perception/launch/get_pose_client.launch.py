from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare






def generate_launch_description():

    #rviz2 configuration
    rviz_config_file = PathJoinSubstitution([FindPackageShare("get_pose_perception"),"rviz","perception_rviz.rviz"])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_perception',
        arguments=["-d",rviz_config_file],
        output='log'
    
    )
    #object detection server
    obj_action_server_node = Node(
        package='simple_grasping',
        executable='basic_grasping_perception_node',
        name='detection_action_srv',
        parameters=[{'debug_topics':True}],
        output='screen'
    
    )

    #launch action client
    obj_action_client_node = Node(
        package='get_pose_perception',
        executable='minimal_action_client',
        name='minimal_action_client',
        output='screen'
    )

    return LaunchDescription([
    
        rviz_node,
        obj_action_server_node,
        obj_action_client_node
    ])