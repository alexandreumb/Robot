'''
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    LaunchConfiguration,
    Command,
    FindExecutable,
    PathJoinSubstitution,
    PythonExpression,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os


def generate_launch_description():

    # ── arguments ─────────────────────────────────────────────────────────────
    declared_arguments = [
        DeclareLaunchArgument(
            "use_sim",
            default_value="false",
            description="Use Gazebo simulation if true, real hardware if false",
        ),
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Launch RViz if true",
        ),
        DeclareLaunchArgument(
            "remap_odometry_tf",
            default_value="true",
            description="Remap odometry to tf frame",
        ),
    ]

    use_sim          = LaunchConfiguration("use_sim")
    gui              = LaunchConfiguration("gui")
    remap_odometry_tf = LaunchConfiguration("remap_odometry_tf")
    world_path = PathJoinSubstitution(
        [get_package_share_directory("example"), "worlds", "my_world.world"]
    )

    # ── robot description ──────────────────────────────────────────────────────
    # passes use_sim to xacro so the correct hardware plugin is selected
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [get_package_share_directory("example"), "urdf", "my_robot.urdf.xacro"]
            ),
            " use_sim:=",
            use_sim,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [get_package_share_directory("example"), "config", "system.yaml"]
    )

    rviz_config_file = PathJoinSubstitution(
        [get_package_share_directory("example"), "rviz/rviz", "rviz.rviz"]
    )

    # ── Gazebo ─────────────────────────────────────────────────────────────────
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py"
            )
        ),
        launch_arguments={
            "gui": "false",
            "world": os.path.join(
                get_package_share_directory("example"), "worlds", "my_world.world"
            ),
        }.items(),
        condition=IfCondition(use_sim),
    )

    # spawn robot into Gazebo from /robot_description topic
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "robot2"],
        output="screen",
        condition=IfCondition(use_sim),
    )

    # ── robot state publisher ──────────────────────────────────────────────────
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[
            robot_description,
            {"use_sim_time": use_sim},
        ],
    )

    # ── ros2_control node ──────────────────────────────────────────────────────
    # in simulation, gazebo_ros2_control handles this — no separate control node needed
    # in real hardware, ros2_control_node runs as normal
    control_node_remapped = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers, {"use_sim_time": use_sim}],
        output="both",
        remappings=[ ("~/robot_description", "/robot_description"), 
                                             ("/ackermann_steering_controller/tf_odometry", "/tf"), ], 
        condition=UnlessCondition(use_sim),  # ← simply don't run in simulation
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers, {"use_sim_time": use_sim}],
        output="both",
        remappings=[ ("~/robot_description", "/robot_description"), ], 
        condition=UnlessCondition(use_sim),  # ← simply don't run in simulation
    ) 
    

    # ── RViz ──────────────────────────────────────────────────────────────────
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(gui),
        parameters=[{"use_sim_time": use_sim}],
    )

    # ── controller spawners ────────────────────────────────────────────────────
    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "ackermann_steering_controller",
            "--controller-manager", "/controller_manager",
        ],
        parameters=[{"timeout": 20.0, "use_sim_time": use_sim}],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
        ],
        parameters=[{"use_sim_time": use_sim}],
    )

    gps_sensor_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "gps_sensor_broadcaster",
            "--controller-manager", "/controller_manager",
            "--controller-type", "gps_sensor_broadcaster/GPSSensorBroadcaster",
        ],
        parameters=[{"use_sim_time": use_sim}],
        condition=UnlessCondition(use_sim),  # ← only on real hardware
    )

    # ── sequencing ─────────────────────────────────────────────────────────────
    # in simulation: wait for spawn_entity before starting controllers
    delay_controllers_after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[robot_controller_spawner],
        ),
        condition=IfCondition(use_sim),
    )

    # in real hardware: start controllers immediately
    delay_controllers_real = TimerAction(
        period=2.0,
        actions=[robot_controller_spawner],
        condition=UnlessCondition(use_sim),
    )

    delay_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    delay_gps_sensor_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[gps_sensor_broadcaster_spawner],
        )
    )

    delayed_rviz = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    tf_relay = Node(
        package='topic_tools',
        executable='relay',
        name='odom_tf_relay',
        arguments=[
            '/ackermann_steering_controller/tf_odometry',
            '/tf'
        ],
        condition=IfCondition(use_sim),
    )

    nodes = [
        gazebo,
        robot_state_publisher_node,
        spawn_entity,
        control_node,
        control_node_remapped,
        delay_controllers_after_spawn,
        delay_controllers_real,
        delay_joint_state_broadcaster,
        delay_gps_sensor_broadcaster,
        delayed_rviz,
        tf_relay,
        
    ]

    return LaunchDescription(declared_arguments + nodes)

'''
from launch import LaunchDescription 
from launch.actions import DeclareLaunchArgument, TimerAction, RegisterEventHandler 
from launch.event_handlers import OnProcessExit 
from launch.conditions import IfCondition, UnlessCondition 
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution 
from ament_index_python.packages import get_package_share_directory 
import os 
import xacro 
from launch_ros.actions import Node 

def generate_launch_description(): 
    use_sim_time = False  # Change to True if using simulation
    declared_arguments = [] 
    declared_arguments.append( 
        DeclareLaunchArgument( "gui", default_value="true", description="Use simulation (Gazebo) clock if true", ) ) 
    
    declared_arguments.append( DeclareLaunchArgument( "remap_odometry_tf", default_value="true", description="Remap odometry to tf frame", ) ) 
    
    #Initialize Arguments 
     
    gui = LaunchConfiguration("gui") 
    remap_odometry_tf = LaunchConfiguration("remap_odometry_tf") 
    
    #Get URDF via xacro 
    
    robot_description_content = Command( 
        [ PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
          PathJoinSubstitution( [get_package_share_directory("example"), "urdf", "my_robot.urdf.xacro"] ), ] ) 
    
    robot_description = {"robot_description": robot_description_content} 
    
    robot_controllers = PathJoinSubstitution( [ get_package_share_directory("example"), "config", "system.yaml", ] ) 
    
    rviz_config_file = PathJoinSubstitution( [ get_package_share_directory("example"), "rviz/rviz", "rviz.rviz", ] )
    
    control_node_remapped = Node( package="controller_manager", executable="ros2_control_node", 
                                 parameters=[robot_controllers, {'use_sim_time': use_sim_time}], output="both", 
                                 remappings=[ ("~/robot_description", "/robot_description"), 
                                             ("/robot_steering_controller/tf_odometry", "/tf"), ], 
                                             condition=IfCondition(remap_odometry_tf), ) 
    '''
    control_node_remapped = Node( package="controller_manager", executable="ros2_control_node", 
                                 parameters=[robot_controllers, {'use_sim_time': use_sim_time}], output="both", 
                                 remappings=[ ("~/robot_description", "/robot_description"), 
                                             ("/ackermann_steering_controller/tf_odometry", "/tf"), ], 
                                             condition=IfCondition(remap_odometry_tf), ) 
    '''
    
    control_node = Node( package="controller_manager", 
                        executable="ros2_control_node", 
                        parameters=[robot_controllers, {'use_sim_time': use_sim_time}], 
                        output="both", 
                        remappings=[ ("~/robot_description", "/robot_description"), ], 
                        condition=UnlessCondition(remap_odometry_tf), ) 
    
    robot_state_publisher_node = Node( package="robot_state_publisher", 
                                      executable="robot_state_publisher", 
                                      output="both", parameters=[robot_description, {'use_sim_time': use_sim_time}], ) 
    
    rviz_node = Node( package="rviz2", executable="rviz2", name="rviz2", output="log", arguments=["-d", rviz_config_file], condition=IfCondition(gui), parameters=[{'use_sim_time': use_sim_time}],) 
    
    joint_state_broadcaster_spawner = Node( package="controller_manager", executable="spawner", arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"], parameters=[{'use_sim_time': use_sim_time}],) 
    
    gps_sensor_broadcaster_spawner = Node( package="controller_manager", executable="spawner", arguments=["gps_sensor_broadcaster", "--controller-manager", "/controller_manager", "--controller-type", "gps_sensor_broadcaster/GPSSensorBroadcaster"], parameters=[{'use_sim_time': use_sim_time}], ) 
    
    '''
    robot_controller_spawner = Node( package="controller_manager", executable="spawner", arguments=["ackermann_steering_controller", "--controller-manager", "/controller_manager"], parameters=[{"timeout": 20.0}, {'use_sim_time': use_sim_time}], ) 
    '''
    robot_controller_spawner = Node( package="controller_manager", executable="spawner", arguments=["robot_steering_controller", "--controller-manager", "/controller_manager"], parameters=[{"timeout": 20.0}, {'use_sim_time': use_sim_time}], ) 

    # Delay rviz 
    
    delayed_rviz_after_joint_state_broadcaster = RegisterEventHandler( event_handler=OnProcessExit( target_action=joint_state_broadcaster_spawner, on_exit=[rviz_node], ) ) 
    
    delay_joint_state_broadcaster = RegisterEventHandler( event_handler=OnProcessExit( target_action=robot_controller_spawner, on_exit=[joint_state_broadcaster_spawner], ) ) 
    
    delay_gps_sensor_broadcaster = RegisterEventHandler( event_handler=OnProcessExit( target_action=robot_controller_spawner, on_exit=[gps_sensor_broadcaster_spawner], ) ) 
    
    nodes = [ 
        control_node, 
        control_node_remapped, 
        robot_state_publisher_node, 
        robot_controller_spawner, 
        delay_joint_state_broadcaster, 
        delay_gps_sensor_broadcaster, 
        #delayed_rviz_after_joint_state_broadcaster, 
        ] 
    
    return LaunchDescription(declared_arguments + nodes)