import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, event, event_handlers
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, Shutdown
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition, LaunchConfigurationEquals, UnlessCondition
import yaml
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.webots_launcher import WebotsLauncher



def load_yaml(package_name, *paths):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, *paths)
    print(absolute_file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        print("error yaml file")
        return None

def load_file(package_name, *paths):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, *paths)
    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
        print("error load file")
        return None
    

def get_ros2_nodes(*args):

    controller_manager_timeout = ['--controller-manager-timeout', '500']
    controller_manager_prefix = 'python.exe' if os.name == 'nt' else ''
    angle_trajectory_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['arm1650_controller', '-c', 'controller_manager'] + controller_manager_timeout
    )


    angle_joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['arm1650_joint_state_broadcaster', '-c', 'controller_manager'] + controller_manager_timeout
    )

    # gripper_controller_spawner_2 = Node(
    #     package='controller_manager',
    #     executable='spawner',
    #     output='screen',
    #     prefix=controller_manager_prefix,
    #     arguments=['gripper_controller', '-c', 'angle/controller_manager'] + controller_manager_timeout
    # )

    angle_control_node = Node(
        package='manipulators_control',
        executable='angle_robot_control',
        output='screen'
    )

    if len(args) == 1:
        return [angle_joint_state_broadcaster_spawner,
                angle_trajectory_controller_spawner,
                launch.actions.RegisterEventHandler(
                    event_handler=event_handlers.OnProcessExit(
                        target_action=angle_trajectory_controller_spawner,
                        on_exit=[args[0]]
                    )
                ),  
    ]
    else:
        return [angle_trajectory_controller_spawner,
                angle_joint_state_broadcaster_spawner]


def generate_launch_description():
    db_arg = DeclareLaunchArgument(
        'db', default_value='False', description='Database flag'
    )

    angle_urdf_file = os.path.join(get_package_share_directory('arm1650_description'), 'resource', 'urdf', 'arm1650.urdf')

    robot_description_config = Command(
        [FindExecutable(name='xacro'), ' ', angle_urdf_file])  
    
    
    robot_description = {'robot_description': robot_description_config}

    robot_description_semantic_config = load_file(
        'arm1650_description', 'resource', 'srdf', 'arm1650.srdf'
    )


    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_config
    }

    kinematics_yaml = load_yaml(
        'arm1650_config', 'resource', 'config', 'kinematics.yaml'
    )

    ompl_planning_pipeline_config = {
        'move_group': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': 'default_planner_request_adapters/AddTimeOptimalParameterization '
                                'default_planner_request_adapters/ResolveConstraintFrames '
                                'default_planner_request_adapters/FixWorkspaceBounds '
                                'default_planner_request_adapters/FixStartStateBounds '
                                'default_planner_request_adapters/FixStartStateCollision '
                                'default_planner_request_adapters/FixStartStatePathConstraints',
            'start_state_max_bounds_error': 0.01,
        }
    }


    joint_limits_yaml = load_yaml(
        'arm1650_config', 'resource', 'config', 'joint_limits.yaml'
    )
    
    ompl_planning_yaml = load_yaml(
        'arm1650_config', 'resource/config/ompl_planning.yaml'
    )
    ompl_planning_pipeline_config['move_group'].update(ompl_planning_yaml)

    moveit_simple_controllers_yaml = load_yaml(
        'arm1650_config', 'resource', 'config', 'moveit_controllers.yaml'
    )

    moveit_controllers = {
        'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager',
        'moveit_simple_controller_manager': moveit_simple_controllers_yaml,
    }

    trajectory_execution = {
        'allow_trajectory_execution': True,
        'allow_goal_duration_margin': True,
        'allow_start_tolerance': True,
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
    }

    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }

    use_sim_time = {'use_sim_time': True}

    run_move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            joint_limits_yaml,
            kinematics_yaml,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            use_sim_time
        ],

    )

    rviz_base = os.path.join(get_package_share_directory('arm1650_config'), 'resource', 'config')
    rviz_config = os.path.join(rviz_base, 'moveit.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            kinematics_yaml,
            use_sim_time
        ],
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description, use_sim_time], 
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='angle_joint_state_publisher',
        output='screen',
        parameters=[{'source_list': ['/joint_states']}]
    )

    db_config = LaunchConfiguration('db')
    mongodb_server_node = Node(
        package='warehouse_ros_mongo',
        executable='mongo_wrapper_ros.py',
        parameters=[
            {'warehouse_port': 33829},
            {'warehouse_host': 'localhost'},
            {'warehouse_plugin': 'warehouse_ros_mongo::MongoDatabaseConnection'},
        ],
        output='screen',
        condition=IfCondition(db_config)
    )


    world = LaunchConfiguration('world')

    webots = WebotsLauncher(
        world=PathJoinSubstitution([get_package_share_directory('webots'), 'resource', 'worlds', world]),
        mode='realtime',
        ros2_supervisor=True
    )

    # gripper_control_node = Node(
    #     package='black5dof_config',
    #     executable='gripper_controller',
    #     output='screen',
    # )

    angle_control_node = Node(
        package='manipulators_control',
        executable='angle_robot_control',
        output='screen'
    )


    package_dir = get_package_share_directory('arm1650_config')
    ros2_control_params = os.path.join(package_dir, 'resource', 'config', 'ros2_controllers.yaml')
    arm1650_description_path = os.path.join(package_dir, 'urdf', 'arm165_config.urdf')
    angle_driver = WebotsController(
        robot_name='ARM165',
        parameters=[
            {'robot_description': arm1650_description_path},
            {'use_sim_time': True},
            {'set_robot_state_publisher': False},
            ros2_control_params
        ],
        respawn=True
    )


    reset_handler = launch.actions.RegisterEventHandler(
        event_handler=event_handlers.OnProcessExit(
            target_action=webots._supervisor,
            on_exit=get_ros2_nodes
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='simulation_world.wbt',
            description="5-dof arm world"
        ),

        webots,
        webots._supervisor,
        angle_driver,
        robot_state_publisher,
        joint_state_publisher_node,
        db_arg,
        rviz_node,
        angle_control_node,

        reset_handler,
        launch.actions.RegisterEventHandler(
            event_handler=event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.UnregisterEventHandler(
                    event_handler=reset_handler.event_handler
                ),
                launch.actions.EmitEvent(event=launch.events.Shutdown())]
            )
        )
        
    ] + get_ros2_nodes(run_move_group_node)
    
    )