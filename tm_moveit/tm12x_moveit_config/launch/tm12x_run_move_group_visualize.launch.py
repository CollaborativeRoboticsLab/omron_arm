############################################################################################### 
#  tm12x_run_move_group.launch.py
#   
#  Various portions of the code are based on original source from 
#  The reference: "https://github.com/moveit/moveit2/tree/main/moveit_ros/moveit_servo/launch"
#  and are used in accordance with the following license.
#  "https://github.com/moveit/moveit2/blob/main/LICENSE.txt"
############################################################################################### 

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

import xacro
import yaml


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except OSError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except OSError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
 
    # Configure robot_description
    description_path = 'tm_description'
    xacro_path = 'tm12x.urdf.xacro'
    moveit_config_path = 'tm12x_moveit_config'    
    srdf_path = 'config/tm12x.srdf'
    rviz_path = '/launch/run_move_group.rviz'     
    
    robot_description_config = xacro.process_file(
        os.path.join(
            get_package_share_directory(description_path),
            'xacro',
            xacro_path,
        )
    )
    robot_description = {'robot_description': robot_description_config.toxml()}

    # SRDF Configuration
    robot_description_semantic_config = load_file(moveit_config_path, srdf_path)
    robot_description_semantic = {'robot_description_semantic': robot_description_semantic_config}

    # Kinematics
    kinematics_yaml = load_yaml(moveit_config_path, 'config/kinematics.yaml')
    robot_description_kinematics = {'robot_description_kinematics': kinematics_yaml}

    # Planning Configuration
    ompl_planning_pipeline_config = {
        'planning_pipelines': ['ompl'],
        'ompl': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            'start_state_max_bounds_error': 0.1,
        },
    }
    ompl_planning_yaml = load_yaml(moveit_config_path, 'config/ompl_planning.yaml')
    ompl_planning_pipeline_config['ompl'].update(ompl_planning_yaml)


    # Joint limits
    joint_limits_yaml = {
        'robot_description_planning': load_yaml(
            moveit_config_path, 'config/joint_limits.yaml'
        )
    }


    # RViz configuration
    rviz_config_file = (
        get_package_share_directory(moveit_config_path) + rviz_path
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        emulate_tty=True,
        arguments=['-d', rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            ompl_planning_pipeline_config,
            joint_limits_yaml,
        ],
    )


    # Launching all the nodes
    return LaunchDescription(
        [
            rviz_node,
        ]
    )
