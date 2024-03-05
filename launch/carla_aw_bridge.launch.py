# Copyright 2023 Gemb Kaljavesi, Technical University of Munich
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

import os
import json

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

class objectParser():
    def __init__(self, configPath):
        self.configPath_ = configPath
        self.topicPrefix_ = "/carla/ego_vehicle"
        self.sensors_ = None
        self.sensorTypeDict_ = {
            "gnss": "sensor.other.gnss",
            "lidar": "sensor.lidar.ray_cast",
            "imu": "sensor.other.imu",
            "camera": "sensor.camera.rgb",
            "odom": "sensor.pseudo.odom",
            "speedo": "sensor.pseudo.speedometer"
        }
        self.readConfig()
    
    def readConfig(self):
        # Function that reads a json config file and returns a dictionary
        with open(self.configPath_, 'r') as f:
            config = json.load(f)
        
        # This function only looks for the sensors of the first vehicle in 
        # the config
        for actor in config["objects"]:
            actor_type = actor["type"].split('.')[0]
            if actor_type == "vehicle" or actor_type == "walker":
                self.sensors_ = actor["sensors"]
                break

        if self.sensors_ is None:
            raise ValueError("No sensors found!")
    
    def getTopics(self, *input_sensor_types):
        # Function that return the topic of one or
        # multiple sensors as a list
        topicList = []
        
        # Function that checks the input sensor types
        for input_sensor_type in input_sensor_types:
            if input_sensor_type not in ["gnss", "lidar", "imu", 
                                        "camera","odom", "speedo"]:
                raise ValueError("Sensor type not supported!")

        for sensor in self.sensors_:
            config_sensor_type = sensor["type"]
            for input_sensor_type in input_sensor_types:
                if self.sensorTypeDict_[input_sensor_type] == config_sensor_type:
                    if input_sensor_type == "camera":
                        topicList.append(sensor["id"] + "/image")
                        topicList.append(sensor["id"] + "/camera_info")
                        continue
                    topicList.append(sensor["id"])

        return topicList

    def getRemaps(self, *input_sensor_types):
        # Functions that return the remappings of one or multiple sensors
        remappings = []

        targetTopics = self.getTopics(*input_sensor_types)
        for topic in targetTopics:
            if topic.split("/")[-1] != "image":
                remappings.append((
                    self.topicPrefix_ + "/" + topic,
                    "/" + topic
                ))
            else:
                remappings.append((
                    self.topicPrefix_ + "/" + topic,
                    "/" + topic + "_raw"
                ))

        return remappings
    def getConversions(self, *input_sensor_types):
        # Converts the remappings to string
        # Output: type: <type>, input: </topic>, output: </topic>;type: <type>, input: </topic>, output: </topic>;
        conversionString = ""
        
        # Function that checks the input sensor types
        for input_sensor_type in input_sensor_types:
            if input_sensor_type not in ["gnss", "lidar", "imu", 
                                        "camera","odom", "speedo"]:
                raise ValueError("Sensor type not supported!")

        for sensor in self.sensors_:
            config_sensor_type = sensor["type"]
            for input_sensor_type in input_sensor_types:
                if self.sensorTypeDict_[input_sensor_type] == config_sensor_type:
                    conversionString += "type: " + input_sensor_type 
                    if input_sensor_type == "camera":
                        conversionString+= ", input: " + self.topicPrefix_ + "/" + sensor["id"] + "/image"
                        conversionString+= ", output: " + "/" + sensor["id"] + "/image"
                        conversionString += ";"
                        continue
                    conversionString += ", input: " + self.topicPrefix_ + "/" + sensor["id"]
                    conversionString += ", output: " + "/" + sensor["id"]
                    conversionString += ";"
        
        
        return conversionString


def launch_setup(context, *args, **kwargs):
    config_path = LaunchConfiguration('objects_definition_file').perform(context)

    object_Parser = objectParser(config_path)
    
    carla_ros_bridge_launch = launch_ros.actions.Node(
        package='carla_ros_bridge',
        executable='bridge',
        name='carla_ros_bridge',
        output='screen',
        emulate_tty='True',
        on_exit=launch.actions.Shutdown(),
        parameters=
        [
            {
                'use_sim_time': True
            },
            {
                'host': launch.substitutions.LaunchConfiguration('host')
            },
            {
                'port': launch.substitutions.LaunchConfiguration('port')
            },
            {
                'timeout': launch.substitutions.LaunchConfiguration('timeout')
            },
            {
                'passive': launch.substitutions.LaunchConfiguration('passive')
            },
            {
                'synchronous_mode': launch.substitutions.LaunchConfiguration(
                    'synchronous_mode')
            },
            {
                'synchronous_mode_wait_for_vehicle_control_command':
                    launch.substitutions.LaunchConfiguration(
                        'synchronous_mode_wait_for_vehicle_control_command')
            },
            {
                'fixed_delta_seconds': launch.substitutions.LaunchConfiguration(
                    'fixed_delta_seconds')
            },
            {
                'town': launch.substitutions.LaunchConfiguration('town')
            },
            {
                'register_all_sensors': launch.substitutions.LaunchConfiguration(
                    'register_all_sensors')
            },
            {
                'ego_vehicle_role_name': launch.substitutions.LaunchConfiguration(
                    'ego_vehicle_role_name')
            }
        ],
        remappings=object_Parser.getRemaps("gnss", "camera", "imu", "lidar"))

    carla_aw_bridge_launch = launch_ros.actions.Node(
            package='carla_autoware_bridge',
            executable='carla_autoware_bridge',
            name='carla_autoware_bridge',
            on_exit=launch.actions.Shutdown(),
            parameters=[
                {
                    'use_sim_time': True
                },
                {
                    'sensors_to_convert': object_Parser.getConversions("lidar")
                }
            ])

    spawn_ego_vehicle = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('carla_autoware_bridge')),
         '/carla_autoware_ego_vehicle.launch.py']))

    manual_control_window = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('carla_manual_control')),
         '/carla_manual_control.launch.py']),
      condition=IfCondition(LaunchConfiguration('view')))

    ackermann_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('carla_ackermann_control')),
            '/carla_ackermann_control.launch.py']))
    
    return [carla_ros_bridge_launch,
            carla_aw_bridge_launch,
            spawn_ego_vehicle,
            manual_control_window,
            ackermann_launch]

def generate_launch_description():
    host_arg = DeclareLaunchArgument(
            name='host',
            default_value='localhost',
            description='IP of the CARLA server')
    port_arg = DeclareLaunchArgument(
            name='port',
            default_value='2000',
            description='TCP port of the CARLA server')
    timeout_arg = DeclareLaunchArgument(
            name='timeout',
            default_value='5',
            description='Time to wait for a successful connection to the CARLA server')
    town_arg = DeclareLaunchArgument(
            name='town',
            default_value='Town01',
            description=('Either use an available CARLA town (eg. "Town01")'
                         'or an OpenDRIVE file (ending in .xodr)'))
    view_arg = DeclareLaunchArgument(
            name='view',
            default_value='false',
            description=('Is third person view window is needed,'
                         'it can be used to manual control as well'))
    passive_arg = launch.actions.DeclareLaunchArgument(
            name='passive',
            default_value='False',
            description=('When enabled, the ROS bridge will take a backseat'
                         'and another client must tick the world (only in synchronous mode)'))
    sync_arg = launch.actions.DeclareLaunchArgument(
            name='synchronous_mode',
            default_value='True',
            description=('Enable/disable synchronous mode.'
                         'If enabled, the ROS bridge waits'
                         'until the expected data is received for all sensors'))
    sync_wait_arg = launch.actions.DeclareLaunchArgument(
            name='synchronous_mode_wait_for_vehicle_control_command',
            default_value='False',
            description=('When enabled, pauses the tick until a vehicle control'
                         'is completed (only in synchronous mode)'))
    fixed_seconds_arg = launch.actions.DeclareLaunchArgument(
            name='fixed_delta_seconds',
            default_value='0.05',
            description='Simulation time (delta seconds) between simulation steps')
    reg_sensors_arg = launch.actions.DeclareLaunchArgument(
            name='register_all_sensors',
            default_value='True',
            description=('Enable/disable the registration of all sensors.'
                         'If disabled, only sensors spawned by the bridge are registered'))
    ego_name_arg = launch.actions.DeclareLaunchArgument(
            name='ego_vehicle_role_name',
            default_value=['ego_vehicle','hero','hero0', 'hero1', 'hero2',
                           'hero3', 'hero4', 'hero5', 'hero6', 'hero7', 'hero8', 'hero9'],
            description='Role names to identify ego vehicles. ')
    objects_arg = launch.actions.DeclareLaunchArgument(
            name='objects_definition_file',
            default_value=get_package_share_directory(
                'carla_autoware_bridge') + '/config/objects.json')  

    ld = LaunchDescription([
        host_arg,
        port_arg,
        timeout_arg,
        town_arg,
        view_arg,
        passive_arg,
        sync_arg,
        sync_wait_arg,
        fixed_seconds_arg,
        reg_sensors_arg,
        ego_name_arg,
        objects_arg,
        OpaqueFunction(function=launch_setup),
    ])
    return ld

if __name__ == '__main__':
    generate_launch_description()
