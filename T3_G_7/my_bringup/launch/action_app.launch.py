from launch import LaunchDescription
from launch.actions import TimerAction, ExecuteProcess
from launch_ros.actions import Node
import os

def generate_launch_description():

    """
    Launch file per il Task 1
    """
    
    ld = LaunchDescription()
 

    #Nodo 1: (Individua gli aruco nella scena) subito
    aruco_detect = Node(
        package="head_scan",
        executable="aruco_scan_publisher"
    )

    #Nodo 2: (Trasforma le coordinate rispetto a base_footprint) dopo 7 secondi, per compensare il tempo di avvio del primo nodo
    aruco_transform = TimerAction(
        period=7.0,
        actions=[
            Node(
                package="head_scan",
                executable="aruco_coord_transformation"
            )
        ]
    )

    #Nodo 3: (Movimento della testa) dopo 10 secondi dall'inizio (3 dal precedente)
    head_movement = TimerAction(
        period=10.0,
        actions=[
            Node(
                package="head_scan",
                executable="head_movement_client"
            )
        ]
    )

    # Apertura della configurazione rviz che mostra le pose trasformate degli aruco e la terna relativa all'end-effector
    # Tenta di usare la config RViz presente nel repo (T2_G_7/urdf_config.rviz); se non trovata, avvia RViz senza config
    this_dir = os.path.dirname(__file__)
    candidate1 = os.path.abspath(os.path.join(this_dir, '..', '..', '..', 'T2_G_7', 'urdf_config.rviz'))
    candidate2 = os.path.abspath(os.path.join(this_dir, '..', '..', 'urdf_config.rviz'))
    rviz_config = candidate1 if os.path.exists(candidate1) else (candidate2 if os.path.exists(candidate2) else None)
    rviz_cmd = ['ros2', 'run', 'rviz2', 'rviz2']
    if rviz_config:
        rviz_cmd += ['-d', rviz_config]

    rviz = TimerAction(
        period=48.0,
        actions=[
            ExecuteProcess(
                cmd=rviz_cmd
            )
        ]
    )

    # Alzata del torso fino a [0.35]
    torso_movement = TimerAction(
        period=50.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'timeout','10','ros2', 'topic', 'pub', '/torso_controller/joint_trajectory',
                    'trajectory_msgs/msg/JointTrajectory', 
                    '{"joint_names": ["torso_lift_joint"], "points": [{"positions": [0.35], "time_from_start": {"sec": 2, "nanosec": 0}}]}'
                ]
            )
        ]
    )

    # Movimento intermedio del braccio fino a [0.05, -0.05, -0.05, -0.05, 0.05, 0.05, 0.05]
    arm_movement_1 = TimerAction(
        period=58.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'timeout','12','ros2', 'topic', 'pub', '/arm_controller/joint_trajectory',
                    'trajectory_msgs/msg/JointTrajectory',
                    '{"joint_names": ["arm_1_joint", "arm_2_joint", "arm_3_joint", "arm_4_joint", "arm_5_joint", "arm_6_joint", "arm_7_joint"], "points": [{"positions": [0.05, -0.05, -0.05, -0.05, 0.05, 0.05, 0.05],"time_from_start": {"sec": 2, "nanosec": 0}}]}'
                
                ]
            )
        ]
    )

    # Movimento finale del braccio fino a [0.07, 0.1, -3.1, 1.36, 2.05, 0.01, -0.05]
    arm_movement_2 = TimerAction(
        period=70.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'timeout','12','ros2', 'topic', 'pub', '/arm_controller/joint_trajectory',
                    'trajectory_msgs/msg/JointTrajectory', 
                    '{"joint_names": ["arm_1_joint", "arm_2_joint", "arm_3_joint", "arm_4_joint", "arm_5_joint", "arm_6_joint", "arm_7_joint"], "points": [{"positions": [0.07, 0.1, -3.1, 1.36, 2.05, 0.01, -0.05],"time_from_start": {"sec": 2, "nanosec": 0}}]}'
                ]
            )
        ]
    )

    # Nodo 4: (Motion Planner per l'inversione cinematica) dopo 83 secondi dall'inizio (subito dopo il movimento del braccio)
    motion_planner = TimerAction(
        period=83.0,
        actions=[
            Node(
                package="head_scan",
                executable="motion_planner_node"
            )
        ]
    )

    # Nodo 5: (State Machine) dopo 90 secondi dall'inizio (7 secondi dal precedente)
    state_machine = TimerAction(
        period=90.0,
        actions=[
            Node(
                package="head_scan",
                executable="state_machine_node"
            )
        ]
    )

    

    #Aggiunge le azioni al LaunchDescription

    
    ld.add_action(aruco_detect)
    ld.add_action(aruco_transform)
    ld.add_action(head_movement)
    ld.add_action(rviz)
    ld.add_action(torso_movement)
    ld.add_action(arm_movement_1)
    ld.add_action(arm_movement_2)
    ld.add_action(motion_planner)
    ld.add_action(state_machine)


    return ld
