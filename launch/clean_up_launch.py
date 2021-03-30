# Copyright 2021 Intelligent Robotics Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory, get_package_prefix

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription,\
    ExecuteProcess, EmitEvent, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, LifecycleNode
from launch_ros.events.lifecycle import ChangeState
import launch.events
import lifecycle_msgs.msg

def generate_launch_description():
    # Get the launch directory
    example_dir = get_package_share_directory('gb_manipulation')
    namespace = LaunchConfiguration('namespace')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace')

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1')
    
    gb_manipulation_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('gb_manipulation'),
            'launch',
            'gb_manipulation_launch.py')))


    # Specify the actions
    clean_up_executor_cmd = LifecycleNode(
        package='clean_up',
        executable='cleanup_executor_node',
        name='cleanup_executor_node')
    emit_event_to_request_that_clean_up_executor_configure_transition = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(clean_up_executor_cmd),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )
    emit_event_to_request_that_clean_up_executor_activate_transition = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(clean_up_executor_cmd),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
        )
    )
    
    # Specify the dependencies
    vision_cmd = LifecycleNode(
      package='clean_up',
      executable='vision_sim_node',
      name='vision')
    emit_event_to_request_that_vision_configure_transition = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(vision_cmd),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(declare_namespace_cmd)

    # Declare the launch options
    ld.add_action(gb_manipulation_cmd)
    ld.add_action(clean_up_executor_cmd)
    ld.add_action(vision_cmd)

    ld.add_action(emit_event_to_request_that_vision_configure_transition)
    ld.add_action(emit_event_to_request_that_clean_up_executor_configure_transition)
    ld.add_action(emit_event_to_request_that_clean_up_executor_activate_transition)

    return ld