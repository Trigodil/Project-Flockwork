"""Stress test: wind, static obstacles, tight formation spacing, and a
real point A to point B transit instead of hovering or orbiting in place.

Deliberately isolates stage 3 (consensus_controller_node) with no mean
field layer, every drone gets the same destination directly.

Kept out of the swarm_control package on purpose, this is a test
scenario, not core control logic.
"""

import math
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

HOVER_HEIGHT = 2.0
# Must match tests/worlds/stress_test.sdf's obstacle_1/2/3 poses.
OBSTACLES = [(-2.0, -1.5, 0.4), (0.0, 1.5, 0.4), (2.0, -1.0, 0.4)]
START_SPREAD = 1.2

# Same X3 UAV base as swarm_control's template, plus a rigidly-joined
# "wind_sail" link with enable_wind set. The drone's real body link lives
# inside the fetched Fuel model, so we can't flip enable_wind on it
# directly, force on this sail transmits through the fixed joint to the
# whole rigid body instead. Verified this actually pushes the drone
# before writing it here (see chat).
DRONE_SDF_TEMPLATE = """<sdf version="1.6">
<model name="{ns}">
  <pose>{x} {y} 0.1 0 0 0</pose>
  <include merge="true">
    <uri>
      https://fuel.gazebosim.org/1.0/OpenRobotics/models/X3 UAV/4
    </uri>
  </include>
  <link name="wind_sail">
    <pose relative_to="X3/base_link">0 0 0 0 0 0</pose>
    <enable_wind>true</enable_wind>
    <inertial>
      <mass>0.01</mass>
      <inertia><ixx>0.0001</ixx><iyy>0.0001</iyy><izz>0.0001</izz></inertia>
    </inertial>
  </link>
  <joint name="wind_sail_joint" type="fixed">
    <parent>X3/base_link</parent>
    <child>wind_sail</child>
  </joint>
  <link name="lidar_link">
    <pose relative_to="X3/base_link">0 0 0 0 0 0</pose>
    <inertial>
      <mass>0.01</mass>
      <inertia><ixx>0.0001</ixx><iyy>0.0001</iyy><izz>0.0001</izz></inertia>
    </inertial>
    <sensor name="lidar_sensor" type="gpu_lidar">
      <always_on>true</always_on>
      <update_rate>10</update_rate>
      <visualize>false</visualize>
      <lidar>
        <scan>
          <horizontal>
            <samples>180</samples>
            <resolution>1</resolution>
            <min_angle>-3.14159</min_angle>
            <max_angle>3.14159</max_angle>
          </horizontal>
        </scan>
        <range>
          <min>0.45</min>
          <max>6.0</max>
          <resolution>0.01</resolution>
        </range>
      </lidar>
    </sensor>
  </link>
  <joint name="lidar_joint" type="fixed">
    <parent>X3/base_link</parent>
    <child>lidar_link</child>
  </joint>
  <plugin filename="gz-sim-multicopter-motor-model-system" name="gz::sim::systems::MulticopterMotorModel">
    <robotNamespace>{ns}</robotNamespace>
    <jointName>X3/rotor_0_joint</jointName>
    <linkName>X3/rotor_0</linkName>
    <turningDirection>ccw</turningDirection>
    <timeConstantUp>0.0125</timeConstantUp>
    <timeConstantDown>0.025</timeConstantDown>
    <maxRotVelocity>800.0</maxRotVelocity>
    <motorConstant>8.54858e-06</motorConstant>
    <momentConstant>0.016</momentConstant>
    <commandSubTopic>gazebo/command/motor_speed</commandSubTopic>
    <actuator_number>0</actuator_number>
    <rotorDragCoefficient>8.06428e-05</rotorDragCoefficient>
    <rollingMomentCoefficient>1e-06</rollingMomentCoefficient>
    <motorSpeedPubTopic>motor_speed/0</motorSpeedPubTopic>
    <rotorVelocitySlowdownSim>10</rotorVelocitySlowdownSim>
    <motorType>velocity</motorType>
  </plugin>
  <plugin filename="gz-sim-multicopter-motor-model-system" name="gz::sim::systems::MulticopterMotorModel">
    <robotNamespace>{ns}</robotNamespace>
    <jointName>X3/rotor_1_joint</jointName>
    <linkName>X3/rotor_1</linkName>
    <turningDirection>ccw</turningDirection>
    <timeConstantUp>0.0125</timeConstantUp>
    <timeConstantDown>0.025</timeConstantDown>
    <maxRotVelocity>800.0</maxRotVelocity>
    <motorConstant>8.54858e-06</motorConstant>
    <momentConstant>0.016</momentConstant>
    <commandSubTopic>gazebo/command/motor_speed</commandSubTopic>
    <actuator_number>1</actuator_number>
    <rotorDragCoefficient>8.06428e-05</rotorDragCoefficient>
    <rollingMomentCoefficient>1e-06</rollingMomentCoefficient>
    <motorSpeedPubTopic>motor_speed/1</motorSpeedPubTopic>
    <rotorVelocitySlowdownSim>10</rotorVelocitySlowdownSim>
    <motorType>velocity</motorType>
  </plugin>
  <plugin filename="gz-sim-multicopter-motor-model-system" name="gz::sim::systems::MulticopterMotorModel">
    <robotNamespace>{ns}</robotNamespace>
    <jointName>X3/rotor_2_joint</jointName>
    <linkName>X3/rotor_2</linkName>
    <turningDirection>cw</turningDirection>
    <timeConstantUp>0.0125</timeConstantUp>
    <timeConstantDown>0.025</timeConstantDown>
    <maxRotVelocity>800.0</maxRotVelocity>
    <motorConstant>8.54858e-06</motorConstant>
    <momentConstant>0.016</momentConstant>
    <commandSubTopic>gazebo/command/motor_speed</commandSubTopic>
    <actuator_number>2</actuator_number>
    <rotorDragCoefficient>8.06428e-05</rotorDragCoefficient>
    <rollingMomentCoefficient>1e-06</rollingMomentCoefficient>
    <motorSpeedPubTopic>motor_speed/2</motorSpeedPubTopic>
    <rotorVelocitySlowdownSim>10</rotorVelocitySlowdownSim>
    <motorType>velocity</motorType>
  </plugin>
  <plugin filename="gz-sim-multicopter-motor-model-system" name="gz::sim::systems::MulticopterMotorModel">
    <robotNamespace>{ns}</robotNamespace>
    <jointName>X3/rotor_3_joint</jointName>
    <linkName>X3/rotor_3</linkName>
    <turningDirection>cw</turningDirection>
    <timeConstantUp>0.0125</timeConstantUp>
    <timeConstantDown>0.025</timeConstantDown>
    <maxRotVelocity>800.0</maxRotVelocity>
    <motorConstant>8.54858e-06</motorConstant>
    <momentConstant>0.016</momentConstant>
    <commandSubTopic>gazebo/command/motor_speed</commandSubTopic>
    <actuator_number>3</actuator_number>
    <rotorDragCoefficient>8.06428e-05</rotorDragCoefficient>
    <rollingMomentCoefficient>1e-06</rollingMomentCoefficient>
    <motorSpeedPubTopic>motor_speed/3</motorSpeedPubTopic>
    <rotorVelocitySlowdownSim>10</rotorVelocitySlowdownSim>
    <motorType>velocity</motorType>
  </plugin>
  <plugin filename="gz-sim-multicopter-control-system" name="gz::sim::systems::MulticopterVelocityControl">
    <robotNamespace>{ns}</robotNamespace>
    <commandSubTopic>gazebo/command/twist</commandSubTopic>
    <enableSubTopic>enable</enableSubTopic>
    <comLinkName>X3/base_link</comLinkName>
    <velocityGain>2.7 2.7 2.7</velocityGain>
    <attitudeGain>2 3 0.15</attitudeGain>
    <angularRateGain>0.4 0.52 0.18</angularRateGain>
    <maximumLinearAcceleration>2 2 2</maximumLinearAcceleration>
    <rotorConfiguration>
      <rotor><jointName>X3/rotor_0_joint</jointName><forceConstant>8.54858e-06</forceConstant><momentConstant>0.016</momentConstant><direction>1</direction></rotor>
      <rotor><jointName>X3/rotor_1_joint</jointName><forceConstant>8.54858e-06</forceConstant><momentConstant>0.016</momentConstant><direction>1</direction></rotor>
      <rotor><jointName>X3/rotor_2_joint</jointName><forceConstant>8.54858e-06</forceConstant><momentConstant>0.016</momentConstant><direction>-1</direction></rotor>
      <rotor><jointName>X3/rotor_3_joint</jointName><forceConstant>8.54858e-06</forceConstant><momentConstant>0.016</momentConstant><direction>-1</direction></rotor>
    </rotorConfiguration>
  </plugin>
  <plugin filename="gz-sim-odometry-publisher-system" name="gz::sim::systems::OdometryPublisher">
    <dimensions>3</dimensions>
  </plugin>
</model>
</sdf>
"""

CONSENSUS_GAIN_ARGS = [
    'interaction_range', 'desired_spacing', 'min_safe_distance', 'barrier_gain', 'tether_gain',
    'alignment_gain', 'accel_feedforward_gain', 'navigation_gain',
    'accel_filter_k', 'max_horizontal_speed', 'max_cmd_accel',
    'obstacle_detection_radius', 'obstacle_safety_margin',
    'obstacle_radial_gain', 'obstacle_tangential_gain',
]


def launch_swarm(context, *args, **kwargs):
    num_drones = int(LaunchConfiguration('num_drones').perform(context))
    start_x = float(LaunchConfiguration('start_x').perform(context))
    start_y = float(LaunchConfiguration('start_y').perform(context))
    dest_x = float(LaunchConfiguration('destination_x').perform(context))
    dest_y = float(LaunchConfiguration('destination_y').perform(context))
    cols = max(1, math.ceil(math.sqrt(num_drones)))
    consensus_gains = {
        name: float(LaunchConfiguration(name).perform(context)) for name in CONSENSUS_GAIN_ARGS
    }
    lidar_params = {
        'use_lidar_sensing': LaunchConfiguration('use_lidar_sensing').perform(context) == 'true',
        'lidar_cluster_gap': float(LaunchConfiguration('lidar_cluster_gap').perform(context)),
        'lidar_min_cluster_points':
            int(LaunchConfiguration('lidar_min_cluster_points').perform(context)),
        'lidar_radius_padding': float(LaunchConfiguration('lidar_radius_padding').perform(context)),
        'lidar_self_exclusion_radius':
            float(LaunchConfiguration('lidar_self_exclusion_radius').perform(context)),
    }

    actions = []
    bridge_args = []
    bridge_remaps = []

    for i in range(num_drones):
        ns = f'x3_{i}'
        # Tight cluster around the start point, not the wide stage-2 grid.
        sx = start_x + (i % cols) * START_SPREAD
        sy = start_y + (i // cols) * START_SPREAD

        actions.append(Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-world', 'stress_test',
                '-string', DRONE_SDF_TEMPLATE.format(ns=ns, x=sx, y=sy),
                '-name', ns,
                '-x', str(sx), '-y', str(sy), '-z', '0.1',
            ],
            output='screen',
        ))

        bridge_args.append(f'/{ns}/gazebo/command/twist@geometry_msgs/msg/Twist@gz.msgs.Twist')
        bridge_args.append(f'/model/{ns}/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry')
        bridge_remaps.append((f'/{ns}/gazebo/command/twist', f'/{ns}/cmd_vel'))
        bridge_remaps.append((f'/model/{ns}/odometry', f'/{ns}/odom'))

        lidar_topic = f'/world/stress_test/model/{ns}/link/lidar_link/sensor/lidar_sensor/scan'
        bridge_args.append(f'{lidar_topic}@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan')
        bridge_remaps.append((lidar_topic, f'/{ns}/scan'))

        neighbor_names = [f'x3_{j}' for j in range(num_drones) if j != i]
        actions.append(Node(
            package='swarm_control',
            executable='consensus_controller_node',
            name='controller',
            namespace=ns,
            parameters=[{
                'neighbor_names': neighbor_names,
                'target_x': dest_x, 'target_y': dest_y, 'target_z': HOVER_HEIGHT,
                'obstacle_x': [o[0] for o in OBSTACLES],
                'obstacle_y': [o[1] for o in OBSTACLES],
                'obstacle_radius': [o[2] for o in OBSTACLES],
                **consensus_gains,
                **lidar_params,
            }],
            output='screen',
        ))

    actions.append(Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=bridge_args,
        remappings=bridge_remaps,
        output='screen',
    ))

    return actions


def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    world_path = os.path.join(os.path.dirname(__file__), '..', 'worlds', 'stress_test.sdf')

    gz_flag = PythonExpression([
        "'-s -r' if '", LaunchConfiguration('headless'), "' == 'true' else '-r'"
    ])

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': [gz_flag, ' ', world_path]}.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('num_drones', default_value='6'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('start_x', default_value='-8.0'),
        DeclareLaunchArgument('start_y', default_value='0.0'),
        DeclareLaunchArgument('destination_x', default_value='8.0'),
        DeclareLaunchArgument('destination_y', default_value='0.0'),
        DeclareLaunchArgument('interaction_range', default_value='5.0'),
        DeclareLaunchArgument('desired_spacing', default_value='1.3'),
        DeclareLaunchArgument('min_safe_distance', default_value='0.9'),
        DeclareLaunchArgument('barrier_gain', default_value='2.0'),
        DeclareLaunchArgument('tether_gain', default_value='0.5'),
        DeclareLaunchArgument('alignment_gain', default_value='0.5'),
        DeclareLaunchArgument('accel_feedforward_gain', default_value='0.25'),
        DeclareLaunchArgument('navigation_gain', default_value='0.6'),
        DeclareLaunchArgument('accel_filter_k', default_value='0.2'),
        DeclareLaunchArgument('max_horizontal_speed', default_value='2.0'),
        DeclareLaunchArgument('max_cmd_accel', default_value='3.0'),
        DeclareLaunchArgument('obstacle_detection_radius', default_value='2.0'),
        DeclareLaunchArgument('obstacle_safety_margin', default_value='0.4'),
        DeclareLaunchArgument('obstacle_radial_gain', default_value='1.5'),
        DeclareLaunchArgument('obstacle_tangential_gain', default_value='1.0'),
        DeclareLaunchArgument('use_lidar_sensing', default_value='false'),
        DeclareLaunchArgument('lidar_cluster_gap', default_value='0.3'),
        DeclareLaunchArgument('lidar_min_cluster_points', default_value='2'),
        DeclareLaunchArgument('lidar_radius_padding', default_value='0.1'),
        DeclareLaunchArgument('lidar_self_exclusion_radius', default_value='0.45'),
        gz_sim,
        OpaqueFunction(function=launch_swarm),
    ])
