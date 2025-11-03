import numpy as np
import roboticstoolbox as rtb
from roboticstoolbox import ERobot
from geometry_msgs.msg import PoseStamped
import rclpy
from std_msgs.msg import String
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from scipy.spatial.transform import Rotation
from spatialmath import SE3
import time
import copy
from movement_primitives.dmp import CartesianDMP
import pytransform3d.trajectories as ptr
import os


class GazeboTrajectoryPublisher(Node):

    """
    Il nodo GazeboTrajectoryPublisher implementa l'inversione cinematica per il raggiungimento della posa finale elaborata dal
    nodo StateMachine, per la movimentazione del braccio tel robot TIAGo.
    """

    def __init__(self):

        super().__init__('gazebo_trajectory_publisher')

        # Carica il modello del robot da urdf (percorso relativo al pacchetto)
        urdf_loc = os.path.join(os.path.dirname(__file__), 'tiago_robot.urdf')
        self.robot = ERobot.URDF(urdf_loc)

        # Crea un oggetto vuoto PoseStamped per la posa finale da raggiungere
        self.final=PoseStamped()

        # Posizioni di pick
        self.pick_points={'1': (0.61,0.125),
                          '2': (0.59,0.1),
                          '3': (0.61,0.075),
                          '4': (0.59,0.05)}
        
        # Posizioni di place
        self.place_points={'1': (0.59,-0.05),
                          '2': (0.61,-0.075),
                          '3': (0.59,-0.1),
                          '4': (0.61,-0.125)}


        self.joint_trajectory = []
        self.index = 0

        # Inizializza la posa corrente di braccio e torso a quella di relax
        self.current_arm_pose=[0.07, 0.1, -3.1, 1.36, 2.05, -0.52, 1.14]
        self.current_torso_pose=[0.35]

        # Crea un publisher e un subscriber per il topic di comunicazione con StateMachine
        self.sm_ik_publisher=self.create_publisher(String,"sm_ik_communication",10)
        self.sm_ik_listener=self.create_subscription(String,"sm_ik_communication",self.next_task_callback,10)

        # Crea un subscriber per il topic delle pose finali comunicate da StateMachine
        self.final_pose=self.create_subscription(PoseStamped,"final_pose",self.final_pose_found,10)

        # Crea un publisher e un subscriber per registrare e pubblicare la posa di braccio e torso
        self.subscriptiona_ = self.create_subscription(JointTrajectory, '/arm_controller/joint_trajectory', self.new_arm_pose, 10)
        self.subscriptiont_ = self.create_subscription(JointTrajectory, '/torso_controller/joint_trajectory', self.new_torso_pose, 10)
        self.publishera_ = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.publishert_ = self.create_publisher(JointTrajectory, '/torso_controller/joint_trajectory', 10)


    # Quando riceve una posa da raggiungere dalla StateMachine, la salva

    def final_pose_found(self,msg):
        self.final=msg

    # Memorizza la posa corrente di braccio e torso, rispettivamente

    def new_arm_pose(self,msg):
       self.current_arm_pose=list(msg.points[0].positions)
       

    def new_torso_pose(self,msg):
        self.current_torso_pose=list(msg.points[0].positions)
        

    # Quando la StateMachine invia un nuovo task da compiere, viene eseguita next_task_callback, che implementa l'inversione
    # cinematica

    def from_pose_stamped_to_t(self,pose:PoseStamped):
        

        pos_arm_pose = np.array([[pose.pose.position.x],
                        [pose.pose.position.y],
                        [pose.pose.position.z]])
                
        quat_arm_pose = np.array([pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w])
        
        rot_arm_pose=Rotation.from_quat(quat_arm_pose).as_matrix()

        # Calcola la posa da raggiungere
        q=np.vstack((np.hstack((rot_arm_pose,pos_arm_pose)),[0,0,0,1]))
        Tq = SE3(q)

        return Tq
    
    def choose_file(self, action_code_pick, action_code_place, current_action_code):
        if action_code_pick=='2a' and action_code_place=='4b':
            if current_action_code=='pick_2a':
                return action_code_pick, '1b', 2
            """elif current_action_code=='place_4b':
                return action_code_pick, '3b', 2"""

        return action_code_pick, action_code_place, 1

    def next_task_callback(self,msg):

        # Divide il task corrente in corrispondenza dello specificatore per il numero di pose intermedie
        actual_msg_lenght=msg.data.split('_num:')

        # A meno che il task non sia gripper o torso, esegue l'inversione
        if len(actual_msg_lenght)>1 or actual_msg_lenght[0]=='relax':

            # La posa di partenza è data dalle pose correnti di troso e braccio
            q0 = np.array(self.current_torso_pose+self.current_arm_pose)
            T0 = SE3(self.robot.fkine(q0))
            
            # Se il task non è di relax elabora il messaggio PoseStamped di posa finale
            if msg.data!='relax':

                pick_place_info=actual_msg_lenght[1].split('_')
                inversion_type=len(pick_place_info)

                if inversion_type>1:

                    n_steps=int(pick_place_info[0])
                    if pick_place_info[1]=='pick':
                        i_action=0
                    elif actual_msg_lenght[1]=='place':
                        i_action=1

                    action_code_pick=str(pick_place_info[2])+'a'
                    action_code_place=str(pick_place_info[3])+'b'
                    

                    #for i_action in range(0,1):
                    for i_action in [0]:

                        if i_action==0:
                            current_action_code='pick_'+action_code_pick
                            
                            

                        elif i_action==1:
                            current_action_code='place_'+action_code_place
                            

                        rep=1
                        action_code_pick, action_code_place, rep=self.choose_file(action_code_pick=action_code_pick, action_code_place=action_code_place, current_action_code=current_action_code)
                        
                        

                          
                        # Estrae il testo dal file come list (percorso relativo al pacchetto)
                        demo_dir = os.path.join(os.path.dirname(__file__), 'tiago_traiettorie_SG', action_code_pick+"-"+action_code_place)
                        demo_path = os.path.join(demo_dir, f"{current_action_code}_rep{rep}.txt")
                        with open(demo_path, "r", encoding="utf-8") as file:
                            text = list([file.read()])

                        # Divide per righe (singole configurazioni dei giunti) ed elimina l'intestazione
                        splitted_text=text[0].split('\n')
                        splitted_text.pop(0)

                        
                        
                        # Per ogni riga crea una lista formata da tempo e configurazione dei giunti in quell'istante,
                        # dividendo in corrispondenza degli spazi

                        

                        movement_list_str=[]

                        for i in range(0,len(splitted_text)-1):
                            # Trascura le righe vuote
                            if splitted_text[i]!='':
                                movement_list_str_element=splitted_text[i].split(' ')


                                movement_list_str.append(movement_list_str_element)

                        # Converte gli array di stringhe in array di float
                        movement_list=np.array(movement_list_str).astype(float)

                        # Prende solo i valori unici
                        _, unique_indices = np.unique(movement_list[:, 0], return_index=True)
                        unique_movements = movement_list[unique_indices]

                        starting_time=unique_movements[0][0]
                        final_time=unique_movements[-1][0]

                        

                        # Calcola l'intervallo di tempo del movimento e il dt
                        movement_time_range=final_time-starting_time
                        dt=movement_time_range/(n_steps-1)

                        # Effettua il campionamento a intervalli di tempo costanti
                        movement_camp=[]
                        
                        """campione = np.linspace(0, len(unique_movements) - 1, 100, dtype=int)
                        movement_camp = unique_movements[campione]


                        for c in movement_camp:
                            list(c).pop(0)
                            list(c).insert(0, 0.35)"""
                        

                        
                        movement=list(unique_movements[0])
                        movement.pop(0)
                        movement.insert(0, 0.35)
                        movement_camp.append(movement)
                        last_dt_memorized=starting_time
                        for i in range(1,len(unique_movements)-1):
                            if unique_movements[i][0]>=last_dt_memorized+dt:
                                last_dt_memorized+=dt
                                movement=list(unique_movements[i-1])
                                movement.pop(0)
                                movement.insert(0, 0.35)
                                movement_camp.append(movement)
                        movement=list(unique_movements[len(unique_movements)-1])
                        movement.pop(0)
                        movement.insert(0, 0.35)
                        movement_camp.append(movement)
                        
                        

                        

                        # Memorizza il movimento campionato
                        #self.pick_place_movements[pick_place_action_i][i_action]=movement_camp
                        # Applica la cinematica diretta (ottengo matrici 4x4)
                        trajectory=[]

                        for joint_state in movement_camp:
                                trajectory_i=SE3(self.robot.fkine(joint_state))
                                trajectory.append(trajectory_i)
                            
                        

                        

                        

                        Y = ptr.pqs_from_transforms(trajectory)
                        T = np.linspace(0, movement_time_range, n_steps)

                        self.dmp = CartesianDMP(execution_time=movement_time_range, dt=dt, n_weights_per_dim=50, smooth_scaling=True)
                        self.dmp.imitate(T, Y)

                        

                        """Verifica che la DMP si "adatta" anche a nuovo Goal"""
                        # === Nuovo goal ===
                        new_goal_pose_stamped=self.final
                        new_goal_pq = ptr.pqs_from_transforms(self.from_pose_stamped_to_t(pose=new_goal_pose_stamped)) # pos + quat
                        new_start_pq = ptr.pqs_from_transforms(self.robot.fkine(self.current_torso_pose+self.current_arm_pose))
                        self.dmp.configure(start_y=new_start_pq, goal_y=new_goal_pq)
                        _, Y_dmp_generalized = self.dmp.open_loop()

                        


                        trajectory_generalized = ptr.transforms_from_pqs(Y_dmp_generalized)

                        

                        joint_trajectory = []
                        q0=self.current_torso_pose+self.current_arm_pose
                        
                        q_curr = self.current_torso_pose+self.current_arm_pose
                        


                        for i in range(0,len(trajectory_generalized)-1):
                            
                            T_se3 = SE3(trajectory_generalized[i])
                            
                            
                            sol = self.robot.ikine_LM(T_se3, q0=q0)
                            
                            if sol.success and (sol is not None):
                                
                                q_curr = sol.q
                                
                                
                            elif (not sol.success) and (sol is not None):
                               
                                q_curr= copy.deepcopy(joint_trajectory[i-1])
                                
                            joint_trajectory.append(np.array(q_curr))


                        

                        self.index=0
                        self.joint_trajectory = np.array(joint_trajectory)
                    
                        

                else:
                    pos_arm_final = np.array([[self.final.pose.position.x],
                            [self.final.pose.position.y],
                            [self.final.pose.position.z]])
                    
                    quat_arm_final = np.array([self.final.pose.orientation.x,
                        self.final.pose.orientation.y,
                        self.final.pose.orientation.z,
                        self.final.pose.orientation.w])
                    
                    rot_arm_final=Rotation.from_quat(quat_arm_final).as_matrix()

                    # Calcola la posa da raggiungere
                    qf=np.vstack((np.hstack((rot_arm_final,pos_arm_final)),[0,0,0,1]))
                    Tf = SE3(qf)

                    # Specifica il numero di pose intermedie
                    N = int(actual_msg_lenght[1])

                    # INVERSIONE CINEMATICA

                    # 1. Traiettoria cartesiana tra le due pose (SE3)
                    Ts = rtb.ctraj(T0, Tf, N)  # genera N pose interpolate

                    # 2. Inverse kinematics per ciascun frame
                    q_traj = []
                    q_curr = q0
                    for T in Ts:
                        sol = self.robot.ik_NR(T, q0=q_curr, pinv=True)
                        
                        if sol is not None and len(sol) > 0:
                            q_curr = sol[0]
                        q_traj.append(q_curr)

                    # Traiiettoria da seguire a partire dall'elemento self.index=0
                    self.joint_trajectory = np.array(q_traj)
                    self.index=0


            else:
            
                # Calcola la posa da raggiungere in caso di relax
                qf = [0.35, 0.07, 0.1, -3.1, 1.36, 2.05, -0.52, 1.14]
                Tf = SE3(self.robot.fkine(qf))

                # Preimposta 50 posizioni intermedie
                N=50
        
                # INVERSIONE CINEMATICA

                # 1. Traiettoria cartesiana tra le due pose (SE3)
                Ts = rtb.ctraj(T0, Tf, N)  # genera N pose interpolate

                # 2. Inverse kinematics per ciascun frame
                q_traj = []
                q_curr = q0
                for T in Ts:
                    sol = self.robot.ik_NR(T, q0=q_curr, pinv=True)
                    
                    if sol is not None and len(sol) > 0:
                        q_curr = sol[0]
                    q_traj.append(q_curr)

                # Traiiettoria da seguire a partire dall'elemento self.index=0
                self.joint_trajectory = np.array(q_traj)
                self.index=0

            #Crea un timer per l'esecuzione dei singoli movimenti
            self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz


    def timer_callback(self):

        # Se i movimenti intermedi sono terminati, distrugge il timer, attende 1 secondo la fine del movimento e invia un 
        # messaggio di termine task su sm_ik_communication, per il coordinamento con la StateMachine. Esce poi dalla funzione
        if self.index >= len(self.joint_trajectory):

            self.destroy_timer(self.timer)
            
            time.sleep(1)

            end_msg=String()
            end_msg.data='terminato'
            self.sm_ik_publisher.publish(end_msg)

            return


        # Pubblica le pose di torso e braccio, rispettivamente, per l'elemento self.index-esimo di self.trajectory
        q = np.array(self.joint_trajectory[self.index]).astype(float)

        msgt = JointTrajectory()
        msgt.joint_names = ['torso_lift_joint']
        torso_point = JointTrajectoryPoint()
        torso_point.positions = [float(q[0])]
        torso_point.time_from_start.sec = 1

        msgt.points.append(torso_point)
        self.publishert_.publish(msgt)

        msga = JointTrajectory()
        msga.joint_names = [
            'arm_1_joint', 'arm_2_joint', 'arm_3_joint',
            'arm_4_joint', 'arm_5_joint', 'arm_6_joint', 'arm_7_joint'
        ]
        arm_point = JointTrajectoryPoint()
        arm_point.positions = [float(qi) for qi in q[1:]]
        arm_point.time_from_start.sec = 1
        
        msga.points.append(arm_point)
        self.publishera_.publish(msga)

        # Aggiorna l'indice per il l'elemento di self.trajectory
        self.index += 1



def main():
    rclpy.init()
    node = GazeboTrajectoryPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
