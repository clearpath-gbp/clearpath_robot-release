#!/usr/bin/env python3

# Software License Agreement (BSD)
#
# @author    Roni Kreinin <rkreinin@clearpathrobotics.com>
# @copyright (c) 2023, Clearpath Robotics, Inc., All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# * Redistributions of source code must retain the above copyright notice,
#   this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# * Neither the name of Clearpath Robotics nor the names of its contributors
#   may be used to endorse or promote products derived from this software
#   without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# Redistribution and use in source and binary forms, with or without
# modification, is not permitted without the express permission
# of Clearpath Robotics.
import os

from clearpath_config.common.types.platform import Platform
from clearpath_config.manipulators.types.arms import Franka, UniversalRobots
from clearpath_config.manipulators.types.grippers import FrankaGripper
from clearpath_config.platform.battery import BatteryConfig
from clearpath_generator_common.common import LaunchFile, Package
from clearpath_generator_common.launch.generator import LaunchGenerator
from clearpath_generator_common.launch.writer import LaunchWriter
from clearpath_generator_robot.launch.sensors import SensorLaunch


class RobotLaunchGenerator(LaunchGenerator):

    def __init__(self, setup_path: str = '/etc/clearpath/') -> None:
        super().__init__(setup_path)

        # Additional packages specific to physical robots
        self.pkg_clearpath_sensors = Package('clearpath_sensors')
        self.pkg_clearpath_hardware_interfaces = Package('clearpath_hardware_interfaces')

        # Filter for MCU IMU
        self.imu_0_filter_node = LaunchFile.Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter_madgwick',
            namespace=self.namespace,
            parameters=[LaunchFile.Variable('imu_filter')],
            remappings=[
                ('imu/data_raw', 'sensors/imu_0/data_raw'),
                ('imu/mag', 'sensors/imu_0/magnetic_field'),
                ('imu/data', 'sensors/imu_0/data'),
                ('/tf', 'tf'),
            ],
        )

        self.imu_0_filter_config = LaunchFile.LaunchArg(
            'imu_filter',
            default_value=os.path.join(self.platform_params_path, 'imu_filter.yaml'),
        )

        # Configure MCU namespace and domain ID
        self.configure_mcu = LaunchFile.Process(
            name='configure_mcu',
            cmd=[
                ['export ROS_DOMAIN_ID=0;'],
                [LaunchFile.Variable("FindExecutable(name='ros2')"),
                 ' service call platform/mcu/configure',
                 ' clearpath_platform_msgs/srv/ConfigureMcu',
                 ' \"{{domain_id: {0},'.format(self.clearpath_config.system.domain_id),
                 ' robot_namespace: \\\'{0}\\\'}}\"'.format(self.namespace)]
            ]
        )

        # Ethernet MicroROS Agent
        self.eth_uros_node = LaunchFile.Node(
            name='micro_ros_agent',
            package='micro_ros_agent',
            executable='micro_ros_agent',
            namespace=self.namespace,
            arguments=['udp4', '--port', '11411'],
        )

        # J100 MicroROS Agent
        self.j100_uros_node = LaunchFile.Node(
            name='micro_ros_agent',
            package='micro_ros_agent',
            executable='micro_ros_agent',
            namespace=self.namespace,
            arguments=['serial', '--dev', '/dev/clearpath/j100'],
        )

        # J100 Navsat driver
        self.nmea_driver_node = LaunchFile.Node(
            package='nmea_navsat_driver',
            executable='nmea_topic_driver',
            name='nmea_topic_driver',
            namespace=self.namespace,
            remappings=[
                ('nmea_sentence', 'sensors/gps_0/nmea_sentence'),
                ('fix', 'sensors/gps_0/fix'),
                ('heading', 'sensors/gps_0/heading'),
                ('time_reference', 'sensors/gps_0/time_reference'),
                ('vel', 'sensors/gps_0/vel'),
            ],
        )

        # Wireless watcher
        self.wireless_watcher_node = LaunchFile.Node(
            package='wireless_watcher',
            executable='wireless_watcher',
            name='wireless_watcher',
            namespace=self.namespace,
            parameters=[
                {
                    'hz': 1.0,
                    'dev': '',
                    'connected_topic': 'platform/wifi_connected',
                    'connection_topic': 'platform/wifi_status',
                }
            ],
            remappings=[('/diagnostics', 'diagnostics'),],
        )

        # Diagnostics launch args
        self.diag_updater_params = LaunchFile.LaunchArg(
            'diagnostic_updater_params',
            default_value=os.path.join(self.platform_params_path, 'diagnostic_updater.yaml'),
        )
        self.diag_aggregator_params = LaunchFile.LaunchArg(
            'diagnostic_aggregator_params',
            default_value=os.path.join(self.platform_params_path, 'diagnostic_aggregator.yaml'),
        )

        self.diagnostic_args = [
            ('namespace', self.namespace),
            ('updater_parameters', LaunchFile.Variable('diagnostic_updater_params')),
            ('aggregator_parameters', LaunchFile.Variable('diagnostic_aggregator_params')),
        ]

        # Diagnostics launch
        clearpath_diagnostics_package = Package('clearpath_diagnostics')
        self.diagnostics_launch = LaunchFile(
            'diagnostics',
            package=clearpath_diagnostics_package,
            args=self.diagnostic_args)

        # Foxglove bridge
        self.foxglove_bridge_params = LaunchFile.LaunchArg(
            'foxglove_bridge_parameters',
            default_value=os.path.join(
                self.platform_params_path,
                'foxglove_bridge.yaml')
        )

        self.foxglove_bridge_args = [
            ('namespace', self.namespace),
            ('parameters', LaunchFile.Variable(
                'foxglove_bridge_parameters'))
        ]

        self.foxglove_bridge_launch = LaunchFile(
            'foxglove_bridge',
            package=clearpath_diagnostics_package,
            args=self.foxglove_bridge_args
        )

        # Battery state
        self.battery_state_estimator = LaunchFile.Node(
            package='clearpath_hardware_interfaces',
            executable='battery_state_estimator',
            name='battery_state_estimator',
            namespace=self.namespace,
            arguments=['-s', setup_path]
        )

        self.battery_state_control = LaunchFile.Node(
            package='clearpath_hardware_interfaces',
            executable='battery_state_control',
            name='battery_state_control',
            namespace=self.namespace,
            arguments=['-s', setup_path]
        )

        # BMS
        self.bms_launch_file = None
        self.bms_node = None

        # Valence BMS
        if (self.clearpath_config.platform.battery.model in
                [BatteryConfig.VALENCE_U24_12XP, BatteryConfig.VALENCE_U27_12XP]):

            can_dev = 'can1'
            bms_id = '0'

            launch_args = self.clearpath_config.platform.battery.launch_args

            if launch_args:
                if 'can_device' in launch_args:
                    can_dev = launch_args['can_device']
                if 'bms_id' in launch_args:
                    bms_id = launch_args['bms_id']

            bms_launch_args = [
                ('namespace', self.namespace),
                ('can_device', can_dev),
                ('bms_id', bms_id),
            ]

            self.bms_launch_file = LaunchFile(
                'bms',
                package=Package('valence_bms_driver'),
                args=bms_launch_args
                )
        # Inventus BMS
        elif (self.clearpath_config.platform.battery.model in
              [BatteryConfig.S_24V20_U1]):

            launch_args = self.clearpath_config.platform.battery.launch_args

            battery_count = 1

            match(self.clearpath_config.platform.battery.configuration):
                case BatteryConfig.S1P2:
                    battery_count = 2
                case BatteryConfig.S1P4:
                    battery_count = 4
                case BatteryConfig.S1P6:
                    battery_count = 6

            inventus_launch_args = [
                    ('namespace', f'{self.namespace}/platform/bms'),
                    ('interface', 'vcan1'),
                    ('battery_count', str(battery_count)),
                    ('master_id', '49'),
                    ('battery_0_id', '49'),
                    ('battery_1_id', '50'),
                    ('battery_2_id', '51'),
                    ('battery_3_id', '52'),
                    ('battery_4_id', '53'),
                    ('battery_5_id', '54'),
            ]

            for i in range(len(inventus_launch_args)):
                key = inventus_launch_args[i][0]
                if key in launch_args:
                    val = launch_args[key]
                    inventus_launch_args[i] = (key, str(val))

            self.bms_node = LaunchFile(
                'canopen_inventus',
                filename='inventus',
                package=Package('canopen_inventus_bringup'),
                args=inventus_launch_args
            )

        # Lighting
        self.lighting_node = LaunchFile.Node(
          package='clearpath_hardware_interfaces',
          executable='lighting_node',
          name='lighting_node',
          namespace=self.namespace,
          parameters=[{'platform': self.platform_model}],
          remappings=[('/diagnostics', 'diagnostics'),],
        )

        # Pinout
        self.pinout_node = LaunchFile.Node(
          package='clearpath_hardware_interfaces',
          executable='pinout_control_node',
          name='pinout_control_node',
          namespace=self.namespace,
          parameters=[{'platform': self.platform_model}],
        )

        # Sevcon
        self.sevcon_node = LaunchFile.Node(
          package='sevcon_traction',
          executable='sevcon_traction_node',
          name='sevcon_traction_node',
          namespace=self.namespace,
          remappings=[('/diagnostics', 'diagnostics')],
        )

        # Puma Multi-Drive Node
        self.puma_node = LaunchFile.Node(
          package='puma_motor_driver',
          executable='multi_puma_node',
          parameters=[os.path.join(self.platform_params_path, 'control.yaml')],
          name='puma_control',
          namespace=self.namespace,
          remappings=[('/diagnostics', 'diagnostics')],
        )

        # BLDC Multi-Drive Node
        self.lynx_node = LaunchFile.Node(
          package='lynx_motor_driver',
          executable='lynx_motor_driver',
          parameters=[os.path.join(self.platform_params_path, 'control.yaml')],
          name='lynx_control',
          namespace=self.namespace,
          remappings=[('/diagnostics', 'diagnostics'),],
        )

        # ROS2 socketcan bridges
        ros2_socketcan_package = Package('clearpath_ros2_socketcan_interface')
        self.can_bridges = []
        for can_bridge in self.clearpath_config.platform.can_bridges.get_all():
            self.can_bridges.append(LaunchFile(
                f'{can_bridge.interface}_receiver',
                filename='receiver',
                package=ros2_socketcan_package,
                args=[
                    ('namespace', self.namespace),
                    ('interface', can_bridge.interface),
                    ('from_can_bus_topic', can_bridge.topic_rx),
                    ('enable_can_fd', str(can_bridge.enaled_can_fd).lower()),
                    ('interval_sec', str(can_bridge.interval)),
                    ('use_bus_time', str(can_bridge.use_bus_time).lower()),
                    ('filters', str(can_bridge.filters)),
                    ('auto_configure', str(can_bridge.auto_configure).lower()),
                    ('auto_activate', str(can_bridge.auto_activate).lower()),
                    ('timeout', str(can_bridge.timeout)),
                    ('transition_attempts', str(can_bridge.transition_attempts)),
                ]
            ))

            self.can_bridges.append(LaunchFile(
                f'{can_bridge.interface}_sender',
                filename='sender',
                package=ros2_socketcan_package,
                args=[
                    ('namespace', self.namespace),
                    ('interface', can_bridge.interface),
                    ('to_can_bus_topic', can_bridge.topic_tx),
                    ('enable_can_fd', str(can_bridge.enaled_can_fd).lower()),
                    ('interval_sec', str(can_bridge.interval)),
                    ('auto_configure', str(can_bridge.auto_configure).lower()),
                    ('auto_activate', str(can_bridge.auto_activate).lower()),
                    ('timeout', str(can_bridge.timeout)),
                    ('transition_attempts', str(can_bridge.transition_attempts)),
                ]
            ))

        # A300 Fan Control Node
        self.a300_fan_control = LaunchFile.Node(
          package='clearpath_hardware_interfaces',
          executable='fan_control_node',
          name='a300_fan_control',
          namespace=self.namespace,
          remappings=[('/diagnostics', 'diagnostics')],
        )

        # A300 SW Low SOC cutoff Node
        self.a300_sw_low_soc_cutoff = LaunchFile.Node(
          package='clearpath_hardware_interfaces',
          executable='sw_low_soc_cutoff_node',
          name='a300_sw_low_soc_cutoff',
          namespace=self.namespace,
        )

        # Components required for each platform
        common_platform_components = [
            self.diag_updater_params,
            self.diag_aggregator_params,
            self.diagnostics_launch,
            self.battery_state_control,
        ]

        if self.clearpath_config.platform.enable_foxglove_bridge:
            common_platform_components.append(self.foxglove_bridge_params)
            common_platform_components.append(self.foxglove_bridge_launch)

        # Only add estimator when no BMS is present
        if self.bms_launch_file is None and self.bms_node is None:
            common_platform_components.append(self.battery_state_estimator)

        if self.clearpath_config.platform.enable_wireless_watcher:
            common_platform_components.append(self.wireless_watcher_node)

        if len(self.can_bridges) > 0:
            common_platform_components.extend(self.can_bridges)

        self.platform_components = {
            Platform.J100: common_platform_components + [
                self.imu_0_filter_node,
                self.imu_0_filter_config,
                self.configure_mcu,
                self.j100_uros_node,
                self.nmea_driver_node
            ],
            Platform.A200: common_platform_components,
            Platform.A300: common_platform_components + [
                self.eth_uros_node,
                self.configure_mcu,
                self.lighting_node,
                self.lynx_node,
                self.a300_fan_control,
                self.a300_sw_low_soc_cutoff,
                self.pinout_node,
            ],
            Platform.W200: common_platform_components + [
                self.imu_0_filter_node,
                self.imu_0_filter_config,
                self.eth_uros_node,
                self.configure_mcu,
                self.lighting_node,
                self.sevcon_node
            ],
            Platform.DD100: common_platform_components + [
                self.imu_0_filter_node,
                self.imu_0_filter_config,
                self.eth_uros_node,
                self.configure_mcu,
                self.lighting_node,
                self.puma_node,
                self.pinout_node,
            ],
            Platform.DO100: common_platform_components + [
                self.imu_0_filter_node,
                self.imu_0_filter_config,
                self.eth_uros_node,
                self.configure_mcu,
                self.lighting_node,
                self.puma_node,
                self.pinout_node,
            ],
            Platform.DD150: common_platform_components + [
                self.imu_0_filter_node,
                self.imu_0_filter_config,
                self.eth_uros_node,
                self.configure_mcu,
                self.lighting_node,
                self.puma_node,
                self.pinout_node,
            ],
            Platform.DO150: common_platform_components + [
                self.imu_0_filter_node,
                self.imu_0_filter_config,
                self.eth_uros_node,
                self.configure_mcu,
                self.lighting_node,
                self.puma_node,
                self.pinout_node,
            ],
            Platform.R100: common_platform_components + [
                self.imu_0_filter_node,
                self.imu_0_filter_config,
                self.eth_uros_node,
                self.configure_mcu,
                self.lighting_node,
                self.puma_node,
            ],
        }

    def generate_sensors(self) -> None:
        sensors_service_launch_writer = LaunchWriter(self.sensors_service_launch_file)
        sensors = self.clearpath_config.sensors.get_all_sensors()

        for sensor in sensors:
            if sensor.launch_enabled:
                sensor_launch = SensorLaunch(
                    sensor,
                    self.namespace,
                    self.sensors_launch_path,
                    self.sensors_params_path,
                )
                # Generate sensor launch file
                sensor_launch.generate()
                # Include sensor launch in top level sensors launch file
                sensors_service_launch_writer.add(sensor_launch.launch_file)

        sensors_service_launch_writer.generate_file()

    def generate_platform(self) -> None:
        platform_service_launch_writer = LaunchWriter(self.platform_service_launch_file)
        platform_service_launch_writer.add(self.platform_launch_file)

        for component in self.platform_components[self.platform_model]:
            platform_service_launch_writer.add(component)

        if self.bms_launch_file:
            platform_service_launch_writer.add(self.bms_launch_file)

        if self.bms_node:
            platform_service_launch_writer.add(self.bms_node)

        platform_service_launch_writer.generate_file()

        platform_extras_service_launch_writer = LaunchWriter(
            self.platform_extras_service_launch_file)
        platform_extras_service_launch_writer.add(self.platform_extras_launch_file)

        if self.clearpath_config.platform.extras.launch:
            extra_launch = LaunchFile(
                name=(os.path.basename(
                    self.clearpath_config.platform.extras.launch['path']
                )).split('.')[0],
                path=os.path.dirname(self.clearpath_config.platform.extras.launch['path']),
                package=Package(self.clearpath_config.platform.extras.launch['package']),
            )
            platform_extras_service_launch_writer.add(extra_launch)

        platform_extras_service_launch_writer.generate_file()

    def generate_manipulators(self) -> None:
        manipulator_service_launch_writer = LaunchWriter(self.manipulators_service_launch_file)
        for arm in self.clearpath_config.manipulators.get_all_arms():
            # Universal Robots Tool Communication
            if arm.MANIPULATOR_MODEL == UniversalRobots.MANIPULATOR_MODEL:
                node = LaunchFile.Node(
                    name=f'{arm.name}_ur_tool_comm',
                    package='ur_robot_driver',
                    executable='tool_communication.py',
                    namespace=self.namespace,
                    parameters=[{
                        'robot_ip': arm.ip,
                        'tcp_port': 54321,
                        'device_name': f'/tmp/{arm.name}_gripper'
                    }],
                )
                manipulator_service_launch_writer.add_node(node)
                # Delay controllers
                self.manipulators_launch_file.args.append(
                    ('control_delay', '1.0')
                )
            # Franka Hand Communication
            if arm.MANIPULATOR_MODEL == Franka.MANIPULATOR_MODEL:
                if arm.gripper:
                    if arm.gripper.MANIPULATOR_MODEL == FrankaGripper.MANIPULATOR_MODEL:
                        node = LaunchFile.Node(
                            name=f'{arm.gripper.name}_controller',
                            package='franka_gripper',
                            executable='franka_gripper_node',
                            namespace=f'{self.namespace}/manipulators',
                            parameters=[{
                                'robot_ip': arm.ip,
                                'joint_names': [
                                    f'{arm.gripper.name}_{arm.gripper.arm_id}_finger_joint1',
                                    f'{arm.gripper.name}_{arm.gripper.arm_id}_finger_joint2'
                                ],
                                'state_publish_rate': 15,  # [Hz]
                                'feedback_publish_rate': 30,  # [Hz]
                                'default_speed': 0.1,  # [m/s]
                                'default_grasp_epsilon': {
                                    'inner': 0.005,  # [m]
                                    'outer': 0.005  # [m]
                                }
                            }],
                            remappings=[
                                ('~/joint_states', f'/{self.namespace}/platform/joint_states')
                            ]
                        )
                        manipulator_service_launch_writer.add_node(node)
        if self.clearpath_config.manipulators.get_all_manipulators():
            manipulator_service_launch_writer.add(self.manipulators_launch_file)
        manipulator_service_launch_writer.generate_file()
