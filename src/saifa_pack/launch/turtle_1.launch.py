from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess,DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    launch_des = LaunchDescription()

    turtlesim = Node(
    package='turtlesim_plus',
    namespace="",
    executable='turtlesim_plus_node.py',
    name='turtlesim'
    )

    launch_des.add_action(turtlesim)

    # interface_name = 'turtlesim/srv/Kill'
    # cmd2 = (
    # f'ros2 service call /{name}/remove_turtle {interface_name} "{{name: \\"/{name}/turtle1\\"}}"'

    # )

    # kill_turtle = ExecuteProcess(
    #     cmd=[cmd2],
    #     shell=True
    # )
    # launch_des.add_action(kill_turtle)
    
    name = "t_name"
    interface_name = 'turtlesim/srv/Spawn'
    cmd2 = (
    f'ros2 service call /spawn_turtle {interface_name} '
    f'"{{x: {0.1}, y: {0.1}, theta: {0.0}, name: {name}}}"'
    )

    set_turtle = ExecuteProcess(
        cmd=[cmd2],
        shell=True
    )
    launch_des.add_action(set_turtle)

    tele_control_turtlesim = Node(
    package='saifa_pack',
    namespace=name,
    executable='teleop_controller.py'
    )

    launch_des.add_action(tele_control_turtlesim)

    pizza_via = Node(
    package='sun_pkg',
    namespace='',
    executable='pizza_viapoint_script.py',
    name = "pizza"
    )

    launch_des.add_action(pizza_via)

    turtlesim2 = Node(
    package='turtlesim_plus',
    namespace="cp",
    executable='turtlesim_plus_node.py',
    name='turtlesim'
    )

    launch_des.add_action(turtlesim2)


    turtle_name = ["Foxy","Noetic","Humble","Iron"]
    cl = "cp"
    interface_name = 'turtlesim/srv/Spawn'
    for i in range(4):
        cmd2 = (
        f'ros2 service call /{cl}/spawn_turtle {interface_name} '
        f'"{{x: {0.1}, y: {0.1}, theta: {0.0}, name: {turtle_name[i]}}}"'
        )

        set_turtle = ExecuteProcess(
            cmd=[cmd2],
            shell=True
        )
        launch_des.add_action(set_turtle)
        cp_turtle = Node(
            package='saifa_pack',
            namespace="cp",
            executable='cp_controller.py',
            name=turtle_name[i] + "_controller",
            parameters=[{'name': turtle_name[i]}]
        )
        launch_des.add_action(cp_turtle)

    sum_n= Node(
    package='saifa_pack',
    namespace="cp",
    executable='sum_controller.py',
    name='sum'
    )

    launch_des.add_action(sum_n)
    
    start_cp = Node(
    package='sun_pkg',
    namespace="cp",
    executable='start_copy_script.py',
    name='start_cp'
    )
    
    launch_des.add_action(start_cp)

    return launch_des