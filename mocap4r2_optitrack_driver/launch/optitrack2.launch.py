# Copyright 2021 Institute for Robotics and Intelligent Machines,
#                Georgia Institute of Technology
# Copyright 2019 Intelligent Robotics Lab
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
#
# Author: Christian Llanes <christian.llanes@gatech.edu>
# Author: David Vargas Frutos <david.vargas@urjc.es>

import os

from ament_index_python.packages import get_package_share_directory
import launch

from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler, TimerAction
from launch.actions import SetEnvironmentVariable, DeclareLaunchArgument
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState

import lifecycle_msgs.msg


def generate_launch_description():

    params_file_path = os.path.join(get_package_share_directory(
      'mocap4r2_optitrack_driver'), 'config', 'mocap_test.yaml')

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1')

    driver_node = LifecycleNode(
        name='mocap4r2_optitrack_driver_node',
        namespace=LaunchConfiguration('namespace'),
        package='mocap4r2_optitrack_driver',
        executable='mocap4r2_optitrack_driver_main',
        output='screen',
        parameters=[LaunchConfiguration('config_file')],
    )

    # Wait for the process to start before sending the configure transition.
    # Using OnProcessStart + TimerAction avoids the race condition where the
    # lifecycle service is not yet discoverable via DDS when launch starts.
    configure_on_start = RegisterEventHandler(
        OnProcessStart(
            target_action=driver_node,
            on_start=[
                TimerAction(
                    period=1.0,
                    actions=[
                        EmitEvent(
                            event=ChangeState(
                                lifecycle_node_matcher=launch.events.matchers.matches_action(
                                    driver_node),
                                transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
                            )
                        )
                    ],
                )
            ],
        )
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(DeclareLaunchArgument('namespace', default_value=''))
    ld.add_action(DeclareLaunchArgument('config_file', default_value=params_file_path))
    ld.add_action(driver_node)
    ld.add_action(configure_on_start)

    return ld
