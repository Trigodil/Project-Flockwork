"""Stage 2 launch file. Spawns N independent drones into an empty world,
bridges each one's topics, and starts one controller node per drone.

Each drone hovers straight up from its own spawn point (same x, y as
spawn, z = 2m) so none of them have to cross another drone's path. No
coordination between drones yet, that is stage 3.
"""

import math
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression

from launch_ros.actions import Node

GRID_SPACING = 3.0
HOVER_HEIGHT = 2.0

# Internal link/joint names inside the fetched X3 UAV model are fixed as
# "X3/..." no matter what the spawned entity is named. Only robotNamespace
# (topic prefix) and the spawn name (entity/odometry prefix) vary per drone.
DRONE_SDF_TEMPLATE = """<sdf version="1.6">
<include>
  <name>{ns}</name>
  <pose>{x} {y} 0.1 0 0 0</pose>
  <uri>
    https://fuel.gazebosim.org/1.0/OpenRobotics/models/X3 UAV/4
  </uri>
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
</include>
</sdf>
"""

# Not active yet. Broadcasts real IMU linear_acceleration per drone
# instead of estimating it from velocity. Needs a <model> wrapping a
# merge="true" include (plain <include><plugin> can't add new links),
# the gz-sim-imu-system world plugin (worlds/swarm.sdf, also commented
# out), and this bridge line per drone:
#   f'/{ns}/imu@sensor_msgs/msg/Imu@gz.msgs.IMU'
#
# DRONE_SDF_TEMPLATE_WITH_IMU = """<sdf version="1.6">
# <model name="{ns}">
#   <pose>{x} {y} 0.1 0 0 0</pose>
#   <include merge="true">
#     <uri>
#       https://fuel.gazebosim.org/1.0/OpenRobotics/models/X3 UAV/4
#     </uri>
#   </include>
#   <link name="imu_link">
#     <pose relative_to="X3/base_link">0 0 0 0 0 0</pose>
#     <sensor name="imu_sensor" type="imu">
#       <always_on>1</always_on>
#       <update_rate>100</update_rate>
#       <topic>{ns}/imu</topic>
#     </sensor>
#   </link>
#   <joint name="imu_joint" type="fixed">
#     <parent>X3/base_link</parent>
#     <child>imu_link</child>
#   </joint>
#   <!-- motor model, velocity control, and odometry publisher plugins
#        go here too, same as DRONE_SDF_TEMPLATE above -->
# </model>
# </sdf>
# """


CONSENSUS_GAIN_ARGS = [
    'interaction_range', 'desired_spacing', 'separation_gain', 'alignment_gain',
    'accel_feedforward_gain', 'navigation_gain', 'accel_filter_k', 'max_horizontal_speed',
]


def launch_swarm(context, *args, **kwargs):
    num_drones = int(LaunchConfiguration('num_drones').perform(context))
    use_mean_field = LaunchConfiguration('use_mean_field').perform(context) == 'true'
    use_consensus = LaunchConfiguration('use_consensus').perform(context) == 'true'
    cols = max(1, math.ceil(math.sqrt(num_drones)))
    consensus_gains = {
        name: float(LaunchConfiguration(name).perform(context)) for name in CONSENSUS_GAIN_ARGS
    }

    actions = []
    bridge_args = []
    bridge_remaps = []
    drone_names = [f'x3_{i}' for i in range(num_drones)]

    for i in range(num_drones):
        ns = f'x3_{i}'
        gx = (i % cols) * GRID_SPACING
        gy = (i // cols) * GRID_SPACING

        actions.append(Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-world', 'swarm',
                '-string', DRONE_SDF_TEMPLATE.format(ns=ns, x=gx, y=gy),
                '-name', ns,
                '-x', str(gx), '-y', str(gy), '-z', '0.1',
            ],
            output='screen',
        ))

        bridge_args.append(f'/{ns}/gazebo/command/twist@geometry_msgs/msg/Twist@gz.msgs.Twist')
        bridge_args.append(f'/model/{ns}/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry')
        bridge_remaps.append((f'/{ns}/gazebo/command/twist', f'/{ns}/cmd_vel'))
        bridge_remaps.append((f'/model/{ns}/odometry', f'/{ns}/odom'))
        # bridge_args.append(f'/{ns}/imu@sensor_msgs/msg/Imu@gz.msgs.IMU')

        if use_consensus:
            neighbor_names = [f'x3_{j}' for j in range(num_drones) if j != i]
            actions.append(Node(
                package='swarm_control',
                executable='consensus_controller_node',
                name='controller',
                namespace=ns,
                parameters=[{
                    'neighbor_names': neighbor_names,
                    'target_x': gx, 'target_y': gy, 'target_z': HOVER_HEIGHT,
                    **consensus_gains,
                }],
                output='screen',
            ))
        else:
            actions.append(Node(
                package='swarm_control',
                executable='single_drone_controller_node',
                name='controller',
                namespace=ns,
                parameters=[{'target_x': gx, 'target_y': gy, 'target_z': HOVER_HEIGHT}],
                output='screen',
            ))

    actions.append(Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=bridge_args,
        remappings=bridge_remaps,
        output='screen',
    ))

    # Global layer, computes each drone's target from the swarm's current
    # collective shape vs a target density, individual PID control in each
    # controller node above is unchanged and just chases whatever target
    # this publishes.
    if use_mean_field:
        actions.append(Node(
            package='swarm_control',
            executable='mean_field_controller_node',
            parameters=[{'drone_names': drone_names}],
            output='screen',
        ))

    return actions


def generate_launch_description():
    pkg_swarm_control = get_package_share_directory('swarm_control')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    world_path = os.path.join(pkg_swarm_control, 'worlds', 'swarm.sdf')

    gz_flag = PythonExpression([
        "'-s -r' if '", LaunchConfiguration('headless'), "' == 'true' else '-r'"
    ])

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': [gz_flag, ' ', world_path]}.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('num_drones', default_value='3'),
        DeclareLaunchArgument('headless', default_value='false',
                               description='Run Gazebo server-only, no GUI.'),
        DeclareLaunchArgument('use_mean_field', default_value='true',
                               description='Run the centralized mean-field density controller.'),
        DeclareLaunchArgument('use_consensus', default_value='false',
                               description='Use stage 3 flocking control instead of stage 1 PID.'),
        DeclareLaunchArgument('interaction_range', default_value='2.5'),
        DeclareLaunchArgument('desired_spacing', default_value='1.5'),
        DeclareLaunchArgument('separation_gain', default_value='1.0'),
        DeclareLaunchArgument('alignment_gain', default_value='0.5'),
        DeclareLaunchArgument('accel_feedforward_gain', default_value='0.25'),
        DeclareLaunchArgument('navigation_gain', default_value='0.6'),
        DeclareLaunchArgument('accel_filter_k', default_value='0.2'),
        DeclareLaunchArgument('max_horizontal_speed', default_value='2.0'),
        gz_sim,
        OpaqueFunction(function=launch_swarm),
    ])
