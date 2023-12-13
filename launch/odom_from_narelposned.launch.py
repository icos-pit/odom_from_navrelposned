import os

import ament_index_python.packages
import launch
import launch_ros.actions


def generate_launch_description():
    config_directory = os.path.join(
        ament_index_python.packages.get_package_share_directory('odom_from_navrelposned'),
        'config')
    params = os.path.join(config_directory, 'conf.yaml')
    odom_from_navrelposned_node = launch_ros.actions.Node(package='odom_from_navrelposned',
                                             executable='odom_from_navrelposned',
                                             name="odom_from_navrelposned",
                                             output='both',
                                             parameters=[params])

    return launch.LaunchDescription([odom_from_navrelposned_node,

                                     launch.actions.RegisterEventHandler(
                                         event_handler=launch.event_handlers.OnProcessExit(
                                             target_action=odom_from_navrelposned_node,
                                             on_exit=[launch.actions.EmitEvent(
                                                 event=launch.events.Shutdown())],
                                         )),
                                     ])
