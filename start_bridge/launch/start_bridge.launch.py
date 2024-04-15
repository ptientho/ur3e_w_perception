from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def generate_launch_description():

    # Set the ROS 1 package and action information
    ros1_package = 'control_msgs'  # Replace with your ROS 1 package name
    ros1_action_type_1 = 'FollowJointTrajectory'
    ros1_action_name_1 = '/scaled_pos_joint_traj_controller/follow_joint_trajectory'

    ros1_action_type_2 =  'GripperCommand'
    ros1_action_name_2 =  '/gripper_controller/gripper_cmd'

    # Set the ros1_bridge parameters
    bridge_package = 'ros1_bridge'
    bridge_executable = 'action_bridge'
    bridge_direction = 'ros1'
    #bridge_arguments = f'{bridge_direction} {ros1_package} {ros1_action_type} {ros1_action_name}'

    '''
    start_bridge_node_1 = ExecuteProcess(
        cmd=['ros2', 'run', bridge_package, bridge_executable, bridge_direction, ros1_package, ros1_action_type_1, ros1_action_name_1, '__name:=action1_bridge'],
        output='screen',
        name='action1'
        )

    start_bridge_node_2 = ExecuteProcess(
        cmd=['ros2', 'run', bridge_package, bridge_executable, bridge_direction, ros1_package, ros1_action_type_2, ros1_action_name_2, '__name:=action2_bridge'],
        output='screen',
        name='action2'
    )
    '''
    start_bridge_node_1 = Node(

        package=bridge_package,
        executable=bridge_executable,
        namespace='action_bridge1',
        arguments=[bridge_direction,ros1_package,ros1_action_type_1,ros1_action_name_1]
    
    
    )

    start_bridge_node_2 = Node(
    
        package=bridge_package,
        executable=bridge_executable,
        namespace='action_bridge2',
        arguments=[bridge_direction,ros1_package,ros1_action_type_2,ros1_action_name_2]
    
    
    )
    

    return LaunchDescription([
    
        start_bridge_node_1,
        start_bridge_node_2
    ])