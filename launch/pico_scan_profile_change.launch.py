from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    param_file = '/home/rsarochand/knapp_ws/src/sick_scan_xd/launch/pico_scan_params.yaml'

    return LaunchDescription([
        Node(name="sick_picoscan_node",
             executable="sick_scansegment_lifecycle_node",
             package="sick_scan_xd",
#             output="screen",
             respawn=False,
             parameters=[param_file, {"scanner_type": "sick_picoscan",
                                      "hostname": "192.168.124.52", # ip address of the lidar
                                      "udp_receiver_ip": "192.168.124.10", # ip address of the receiver (i.e. the ip of the computer running sick_generic_caller)
                                      }
                         ]) #,
#        ExecuteProcess(
#            cmd=[[
#                "bash /home/arti/ros2_ws/src/picoscan_profile_change/change_config_loop.sh"
#            ]],
#            shell=True
#        ),
#        ExecuteProcess(
#            cmd=[[
#                 'ros2 service call /ColaMsg sick_scan_xd/srv/ColaMsgSrv "{request: \'sMN mCLsetscancfglist 05\'}" '
#            ]],
#            shell=True
#        )
    ])
