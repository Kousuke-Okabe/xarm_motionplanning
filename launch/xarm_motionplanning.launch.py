import os

from launch import LaunchDescription
from launch_ros.actions import Node
from uf_ros_lib.moveit_configs_builder import MoveItConfigsBuilder
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import OpaqueFunction
from uf_ros_lib.uf_robot_utils import get_xacro_content
from uf_ros_lib.uf_robot_utils import generate_ros2_control_params_temp_file
from ament_index_python import get_package_share_directory

def launch_setup(context, *args, **kwargs):
    # xacro_file = LaunchConfiguration('xacro_file', default=PathJoinSubstitution([FindPackageShare('xarm_description'), 'urdf', 'xarm_device.urdf.xacro']))
    dof = LaunchConfiguration('dof', default=6)
    robot_type = LaunchConfiguration('robot_type', default='xarm')
    prefix = LaunchConfiguration('prefix', default='')
    hw_ns = LaunchConfiguration('hw_ns', default='xarm')
    limited = LaunchConfiguration('limited', default=False)
    effort_control = LaunchConfiguration('effort_control', default=False)
    velocity_control = LaunchConfiguration('velocity_control', default=False)
    model1300 = LaunchConfiguration('model1300', default=False)
    robot_sn = LaunchConfiguration('robot_sn', default='')
    attach_to = LaunchConfiguration('attach_to', default='world')
    attach_xyz = LaunchConfiguration('attach_xyz', default='"0 0 0"')
    attach_rpy = LaunchConfiguration('attach_rpy', default='"0 0 0"')
    kinematics_suffix = LaunchConfiguration('kinematics_suffix', default='')
    ros2_control_plugin = LaunchConfiguration('ros2_control_plugin', default='uf_robot_hardware/UFRobotSystemHardware')
    add_gripper = LaunchConfiguration('add_gripper', default=False)
    add_vacuum_gripper = LaunchConfiguration('add_vacuum_gripper', default=False)
    add_bio_gripper = LaunchConfiguration('add_bio_gripper', default=False)
    # joint_states_remapping = LaunchConfiguration('joint_states_remapping', default='joint_states')
    add_realsense_d435i = LaunchConfiguration('add_realsense_d435i', default=False)
    add_d435i_links = LaunchConfiguration('add_d435i_links', default=True)
    add_other_geometry = LaunchConfiguration('add_other_geometry', default=False)
    geometry_type = LaunchConfiguration('geometry_type', default='box')
    geometry_mass = LaunchConfiguration('geometry_mass', default=0.1)
    geometry_height = LaunchConfiguration('geometry_height', default=0.1)
    geometry_radius = LaunchConfiguration('geometry_radius', default=0.1)
    geometry_length = LaunchConfiguration('geometry_length', default=0.1)
    geometry_width = LaunchConfiguration('geometry_width', default=0.1)
    geometry_mesh_filename = LaunchConfiguration('geometry_mesh_filename', default='')
    geometry_mesh_origin_xyz = LaunchConfiguration('geometry_mesh_origin_xyz', default='"0 0 0"')
    geometry_mesh_origin_rpy = LaunchConfiguration('geometry_mesh_origin_rpy', default='"0 0 0"')
    geometry_mesh_tcp_xyz = LaunchConfiguration('geometry_mesh_tcp_xyz', default='"0 0 0"')
    geometry_mesh_tcp_rpy = LaunchConfiguration('geometry_mesh_tcp_rpy', default='"0 0 0"')

    # robot_description = LaunchConfiguration('robot_description', default='')

    # if not robot_description.perform(context):
    #     # robot_description
    #     robot_description = {
    #         'robot_description': get_xacro_content(
    #             context,
    #             xacro_file=xacro_file,
    #             dof=dof,
    #             robot_type=robot_type,
    #             prefix=prefix,
    #             hw_ns=hw_ns,
    #             limited=limited,
    #             effort_control=effort_control,
    #             velocity_control=velocity_control,
    #             model1300=model1300,
    #             robot_sn=robot_sn,
    #             attach_to=attach_to,
    #             attach_xyz=attach_xyz,
    #             attach_rpy=attach_rpy,
    #             kinematics_suffix=kinematics_suffix,
    #             ros2_control_plugin=ros2_control_plugin,
    #             add_gripper=add_gripper,
    #             add_vacuum_gripper=add_vacuum_gripper,
    #             add_bio_gripper=add_bio_gripper,
    #             add_realsense_d435i=add_realsense_d435i,
    #             add_d435i_links=add_d435i_links,
    #             add_other_geometry=add_other_geometry,
    #             geometry_type=geometry_type,
    #             geometry_mass=geometry_mass,
    #             geometry_height=geometry_height,
    #             geometry_radius=geometry_radius,
    #             geometry_length=geometry_length,
    #             geometry_width=geometry_width,
    #             geometry_mesh_filename=geometry_mesh_filename,
    #             geometry_mesh_origin_xyz=geometry_mesh_origin_xyz,
    #             geometry_mesh_origin_rpy=geometry_mesh_origin_rpy,
    #             geometry_mesh_tcp_xyz=geometry_mesh_tcp_xyz,
    #             geometry_mesh_tcp_rpy=geometry_mesh_tcp_rpy,
    #         )
    #     }
    # else:
    #     robot_description = yaml.load(robot_description.perform(context), Loader=yaml.FullLoader)

    controllers_name = 'fake_controllers'
    mesh_suffix = LaunchConfiguration('mesh_suffix', default='stl')
    ros_namespace = LaunchConfiguration('ros_namespace', default='').perform(context)
    
    xarm_type = '{}{}'.format(robot_type.perform(context), dof.perform(context) if robot_type.perform(context) in ('xarm', 'lite') else '')

    ros2_control_params = generate_ros2_control_params_temp_file(
        os.path.join(get_package_share_directory('xarm_controller'), 'config', '{}_controllers.yaml'.format(xarm_type)),
        prefix=prefix.perform(context), 
        add_gripper=add_gripper.perform(context) in ('True', 'true'),
        add_bio_gripper=add_bio_gripper.perform(context) in ('True', 'true'),
        ros_namespace=ros_namespace,
        robot_type=robot_type.perform(context)
    )

    moveit_config = MoveItConfigsBuilder(
        context=context,
        controllers_name=controllers_name,
        dof=dof,
        robot_type=robot_type,
        prefix=prefix,
        hw_ns=hw_ns,
        limited=limited,
        effort_control=effort_control,
        velocity_control=velocity_control,
        model1300=model1300,
        robot_sn=robot_sn,
        attach_to=attach_to,
        attach_xyz=attach_xyz,
        attach_rpy=attach_rpy,
        mesh_suffix=mesh_suffix,
        kinematics_suffix=kinematics_suffix,
        ros2_control_plugin=ros2_control_plugin,
        ros2_control_params=ros2_control_params,
        add_gripper=add_gripper,
        add_vacuum_gripper=add_vacuum_gripper,
        add_bio_gripper=add_bio_gripper,
        add_realsense_d435i=add_realsense_d435i,
        add_d435i_links=add_d435i_links,
        add_other_geometry=add_other_geometry,
        geometry_type=geometry_type,
        geometry_mass=geometry_mass,
        geometry_height=geometry_height,
        geometry_radius=geometry_radius,
        geometry_length=geometry_length,
        geometry_width=geometry_width,
        geometry_mesh_filename=geometry_mesh_filename,
        geometry_mesh_origin_xyz=geometry_mesh_origin_xyz,
        geometry_mesh_origin_rpy=geometry_mesh_origin_rpy,
        geometry_mesh_tcp_xyz=geometry_mesh_tcp_xyz,
        geometry_mesh_tcp_rpy=geometry_mesh_tcp_rpy,
    ).to_moveit_configs()

    xarm_motionplanning_node = Node(
        package='xarm_motionplanning',
        executable='xarm_motionplanning_node',
        name='xarm_motionplanning_node',
        parameters=[
            moveit_config.to_dict()
            # robot_description
        ]
    )

    return [
        xarm_motionplanning_node
    ]


def generate_launch_description():
    return LaunchDescription([
    OpaqueFunction(function=launch_setup)
])