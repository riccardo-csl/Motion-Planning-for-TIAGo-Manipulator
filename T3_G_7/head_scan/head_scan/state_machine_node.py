import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from scipy.spatial.transform import Rotation
from std_msgs.msg import String
from spatialmath import SE3
import numpy as np
import time
import copy
import os



class StateMachine(Node):

    """
    Il nodo StateMachine implementa la macchina a stati per lo svolgimento di un task per il braccio robotico di TIAGo, come
    specificato nella catena di esecuzione self.execution_list. La classe permette di specificare 100 pose PoseStamped in poses
    e utilizzarle per eseguire i movimenti, specificando in stringhe di testo offset e rotazioni lungo i tre assi cartesiani.
    Comunica sul topic sm_ik_communication con il nodo per l'inversione cinematica GazeboTrajectoryPublisher.

    I messaggi da inserire nella execution_list sono così composti:

    1. Raggiungimento della posa i con offset (facoltativi, necessarie se presenti le rotazioni), rotazioni (facoltative,
    nell'oridine in cui devono essere effettuate) e numero di posizioni num intermedie per il raggiungimento della posa 
    finale:
        Esempio con sola posa:              'pose=2_num:50'
        Esempio con soli offset:            'pose=2_offset:x=0,z=-0.1_num:50'
        Esempio con sole rotazioni:         'pose=2_offset:x=0,y=0,z=0_rot:Rz=-0.5,Rx=0.5_num:50'
        Esempio con offset e rotazioni:     'pose=2_offset:x=0,y=-0.147,z=-0.1_rot:Rz=-0.5,Rx=0.5_num:50'

    2. Chiusura o apertura del gripper, specificando la posa di entrambe le dita, con eventuale oggetto da afferrare:
        Esempio con oggetto:    'gripper_0.041_0.041_cocacola'
        Esempio senza oggetto:  'gripper_0.044_0.044'

    3. Alzare o abbassare il torso di n rispetto alla posa corrente (movimento relativo) o fino ad una posizione (movimento
    assoluto):
        Esempio relativo:   'torso_rel:-0.05'
        Esempio assoluto:   'torso_0.35'

    4. Ritorno nella posizione di relax con 'relax'.

    Gli offset e i movimenti di gripper e torso sono espressi in metri, le rotazioni in fattori moltiplicativi di pigreco
    (Rx=0.5 corrisponde a rotazione di pigreco/2 su x)
    """

    def __init__(self):

        super().__init__('state_machine_node')

        # Specifica l'oggetto che il gripper deve afferrare utilizzando l'apposito service
        self.gripper_object=None

        # Crea un array di 100 elementi None dove possono essere inseriti oggetti PoseStamped per l'esecuzione del task
        self.poses=100*[None]

        self.poses[1]=PoseStamped()
        self.poses[1].header.frame_id='base_footprint'
        self.poses[1].pose.position.x=0.6002861680675554
        self.poses[1].pose.position.y= 0.23529248455606439
        self.poses[1].pose.position.z=0.9752088478023124
        self.poses[1].pose.orientation.x=-0.0017252968069655743
        self.poses[1].pose.orientation.y=0.0002985299068050842
        self.poses[1].pose.orientation.z=0.9999523052384054
        self.poses[1].pose.orientation.w=-0.008190132068596201

        self.poses[2]=PoseStamped()
        self.poses[2].header.frame_id='base_footprint'
        self.poses[2].pose.position.x=0.5982224669085482
        self.poses[2].pose.position.y= 0.08778918347181165
        self.poses[2].pose.position.z=0.8914147481401276
        self.poses[2].pose.orientation.x=-0.00039997619572689814
        self.poses[2].pose.orientation.y=0.0003586430422672276
        self.poses[2].pose.orientation.z=0.9999433885056647
        self.poses[2].pose.orientation.w=-0.008508395468331034

        self.poses[3]=PoseStamped()
        self.poses[3].header.frame_id='base_footprint'
        self.poses[3].pose.position.x=0.6054543286891046
        self.poses[3].pose.position.y=-0.26800914639661044
        self.poses[3].pose.position.z=0.7380245225984816
        self.poses[3].pose.orientation.x=-0.0023147736441989975
        self.poses[3].pose.orientation.y=0.006649893276948308
        self.poses[3].pose.orientation.z=0.9999505934625172
        self.poses[3].pose.orientation.w=-0.0029248570202548814

        self.poses[4]=PoseStamped()
        self.poses[4].header.frame_id='base_footprint'
        self.poses[4].pose.position.x=0.6053373109739724
        self.poses[4].pose.position.y=-0.11584638567688081
        self.poses[4].pose.position.z=0.7376136104999993
        self.poses[4].pose.orientation.x=-0.005588458503119197
        self.poses[4].pose.orientation.y= 0.0010918223889838973
        self.poses[4].pose.orientation.z=0.9999394587078999
        self.poses[4].pose.orientation.w=-0.0067213686365714675

        # Sequenza di esecuzione dei comandi per lo svolgimento del task 2
        self.execution_list=np.array(['gripper_0.044_0.044',
                                    'pose=2_offset:x=0,y=-0.147,z=-0.1_rot:Rz=-0.5,Rx=0.5_num:50',
                                    'pose=4_offset:x=0,y=0,z=0.15_rot:Ry=0.5,Rx=-0.5_num:50',
                                    'pose=2_offset:x=0.02,y=0.022,z=0_rot:Ry=0.5,Rx=-0.5_num:5',
                                    'pose=2_offset:x=0.02,y=0.022,z=-0.08_rot:Ry=0.5,Rx=-0.5_num:5',
                                    'gripper_0.041_0.041_cocacola',
                                    'pose=2_offset:x=0.02,y=0.022,z=-0.07_rot:Ry=0.5,Rx=-0.5_num:5',
                                    'pose=4_offset:x=0.12,y=0.022,z=0.11_rot:Ry=0.5,Rx=-0.5_num:20',
                                    'pose=3_offset:x=0.01,y=0.01,z=0.08_rot:Ry=0.5,Rx=-0.5_num:20',
                                    'pose=3_offset:x=0.01,y=0.01,z=0.073_rot:Ry=0.5,Rx=-0.5_num:5',
                                    'gripper_0.044_0.044_cocacola',
                                    'pose=3_offset:x=0.01,y=0.01,z=0.15_rot:Ry=0.5,Rx=-0.5_num:10',
                                    'pose=4_offset:x=0.12,y=0.022,z=0.15_rot:Ry=0.5,Rx=-0.5_num:30',
                                    'pose=2_offset:x=-0.007,y=-0.177,z=0.1_rot:Rz=-0.5,Rx=0.5_num:30',
                                    'pose=1_offset:x=-0.007,y=-0.02,z=-0.105_rot:Rz=-0.5,Rx=0.5_num:20',
                                    'gripper_0.041_0.041_pringles',
                                    'pose=1_offset:x=0,y=-0.02,z=-0.095_rot:Rz=-0.5,Rx=0.5_num:5',
                                    'pose=4_offset:x=0.1,y=0.1,z=0.235_rot:Rz=-0.5,Rx=0.5_num:20',
                                    'pose=4_offset:x=-0.04,y=0,z=0.235_rot:Rz=-0.5,Rx=0.5_num:20',
                                    'pose=4_offset:x=-0.04,y=0,z=0.235_rot:Rz=-0.5,Rx=0.5,Ry=-0.25_num:20',
                                    'pose=4_offset:x=-0.04,y=0,z=0.133_rot:Rz=-0.5,Rx=0.5,Ry=-0.25_num:30',
                                    'gripper_0.044_0.044_pringles',
                                    'torso_0.35'])

        # Tiene il numero del task corrente nella self.execution_list
        self.current_task=0

        # Diventa True quando il nodo di inversione cinematica comunica di aver terminato il task precedente
        self.next_task=True

        # Inizializza la posa corrente di torso e gripper a quella di relax
        self.current_gripper_pose=[0.0, 0.0]
        self.current_torso_pose=[0.35]

        # Implementano il movimento relativo del gripper
        self.closing=True   # True se il gripper si sta chiudendo
        self.equal=True     # True se viene inviato un comando al gripper che non ne modifica la posa

        # Crea un publisher e un subscriber per il topic sm_ik_communication, per la comunicazione con il nodo che implementa
        # l'inversione cinematica
        self.sm_ik_publisher=self.create_publisher(String,"sm_ik_communication",10)
        self.sm_ik_listener=self.create_subscription(String,"sm_ik_communication",self.next_task_callback,10)
        
        # Crea un publisher per la posa da raggiungere, che verrà inviata all'inversione cinematica
        self.final_pose=self.create_publisher(PoseStamped,"final_pose",10)

        # Crea un publisher e un subscriber per registrare e pubblicare la posa di gripper e torso
        self.subscription_gripper = self.create_subscription(JointTrajectory, "/gripper_controller/joint_trajectory", self.new_gripper_pose, 10)
        self.subscription_torso = self.create_subscription(JointTrajectory, "/torso_controller/joint_trajectory", self.new_torso_pose, 10)
        self.publisher_gripper=self.create_publisher(JointTrajectory,"/gripper_controller/joint_trajectory",10)
        self.publisher_torso=self.create_publisher(JointTrajectory,"/torso_controller/joint_trajectory",10)
        
        """# Crea la subscription ai topic delle quattro pose trasformate dei 4 aruco utili
        self.aruco_1_pose_sub=self.create_subscription(PoseStamped,"aruco_1_pose_transformed",self.aruco_1_pose_found_callback,10)
        self.aruco_2_pose_sub=self.create_subscription(PoseStamped,"aruco_2_pose_transformed",self.aruco_2_pose_found_callback,10)
        self.aruco_3_pose_sub=self.create_subscription(PoseStamped,"aruco_3_pose_transformed",self.aruco_3_pose_found_callback,10)
        self.aruco_4_pose_sub=self.create_subscription(PoseStamped,"aruco_4_pose_transformed",self.aruco_4_pose_found_callback,10)
        """
        # Crea un timer per l'esecuzione della catena di esecuzione
        self.timer_to_start=self.create_timer(1.0,self.execution_chain)

    
    # Aggiorna la posa corrente di gripper e torso, rispettivamente

    def new_gripper_pose(self,msg):
       self.current_gripper_pose=list(msg.points[0].positions)

    def new_torso_pose(self,msg):
        self.current_torso_pose=list(msg.points[0].positions)


    # Quando la posa dell'aruco con id=n viene rilevata, la salva nel n-esimo elemento del array poses e distrugge la subscription
    
    def aruco_1_pose_found_callback(self,msg):
        self.poses[1]=msg
        self.destroy_subscription(self.aruco_1_pose_sub)

    def aruco_2_pose_found_callback(self,msg):
        self.poses[2]=msg
        self.destroy_subscription(self.aruco_2_pose_sub)

    def aruco_3_pose_found_callback(self,msg):
        self.poses[3]=msg
        self.destroy_subscription(self.aruco_3_pose_sub)

    def aruco_4_pose_found_callback(self,msg):
        self.poses[4]=msg
        self.destroy_subscription(self.aruco_4_pose_sub)


    # Quando viene pubblicato 'terminato' su sm_ik_communication, ossia quando l'inversione cinematica è terminata, permette
    # di eseguire il task successivo

    def next_task_callback(self,msg):
        if msg.data=='terminato':
            self.next_task=True

    
    # Elabora il task corrente

    def execution_chain(self):

        # Esegue l'elaborazione solo dopo aver registrato le pose di quattro aruco utili
        #if (self.poses[1] is not None) and (self.poses[2] is not None) and (self.poses[3] is not None) and (self.poses[4] is not None):

            # Esegue l'elaborazione solo se l'inversione cinematica ha terminato di svolgere il task precedente
        if self.next_task is True:

            self.next_task=False    # La macchina a stati è occupata

            # Prosegue solo se sono presenti ulteriori task nella execution_list
            if self.current_task<len(self.execution_list):
                
                # Divide il task corrente in corrispondenza del carattere '_'
                current_task_splitted=self.execution_list[self.current_task].split('_')

                # Se la prima parola del task NON è né torso né gripper, esegue le azioni relative allo spostamento
                if current_task_splitted[0]!='torso' and current_task_splitted[0]!='gripper':

                    self.get_logger().info(f"Esecuzione del task {self.current_task+1} di {len(self.execution_list)} iniziata: inversione cinematica in corso...")
                    
                    # Esegue decode_msg per calcolare la posa finale a partire dalla stringa descrittiva del task corrente
                    self.final=self.decode_msg()

                    # Pubblica la posa da raggiungere
                    self.final_pose.publish(self.final)

                    # Attende 1 secondo per assicurarsi che l'inversione abbia ricevuto la posa
                    time.sleep(1)

                # Invia il task come da execution_list all'inversione cinematica
                self.msg_state=String()
                self.msg_state.data=self.execution_list[self.current_task]
                self.sm_ik_publisher.publish(self.msg_state)

                # Esegue le azioni per il gripper
                if current_task_splitted[0]=='gripper':
                    
                    # Estrae dalla stringa la posa da raggiungere
                    self.gripper_left_pose=float(current_task_splitted[1])
                    self.gripper_right_pose=float(current_task_splitted[2])

                    # Se è specificato un oggetto da prendere, aggiorna gripper_object
                    if len(current_task_splitted)==4:
                        self.gripper_object=str(current_task_splitted[3])

                    # Rileva se il gripper si deve aprire, chiudere o se non deve fare nulla, rispettivamente
                    if self.gripper_left_pose>float(self.current_gripper_pose[0]) or self.gripper_right_pose>float(self.current_gripper_pose[1]):
                        self.closing=False
                        self.equal=False
                        self.get_logger().info(f"Esecuzione del task {self.current_task+1} di {len(self.execution_list)} iniziata: apertura del gripper in corso...")
                    
                    elif self.gripper_left_pose<float(self.current_gripper_pose[0]) or self.gripper_right_pose<float(self.current_gripper_pose[1]):
                        self.closing=True
                        self.equal=False
                        self.get_logger().info(f"Esecuzione del task {self.current_task+1} di {len(self.execution_list)} iniziata: chiusura del gripper in corso...")
                    
                    else:
                        self.equal=True
                        self.get_logger().info(f"Esecuzione del task {self.current_task+1} di {len(self.execution_list)} iniziata e terminata: gripper già in posizione")
                    

                    # Se il gripper si deve chiudere o aprire pubblica la posa finale
                    if not self.equal:
                        
                        # Crea un oggetto JointTrajectory per la posa da raggiungere del gripper, lo pubblica attende 1
                        # secondo che termini l'azione

                        self.gripper_msg = JointTrajectory()
                        self.gripper_msg.joint_names = [
                            'gripper_left_finger_joint', 'gripper_right_finger_joint'
                        ]
                        self.gripper_point = JointTrajectoryPoint()
                        self.gripper_point.positions = [self.gripper_left_pose,self.gripper_right_pose]
                        self.gripper_point.time_from_start.sec = 1

                        self.gripper_msg.points.append(self.gripper_point)

                        self.publisher_gripper.publish(self.gripper_msg)

                        time.sleep(1)

                        # Se è specificato un oggetto, esegue l'attach o il detach
                        if len(current_task_splitted)==4:

                            # Se si sta chiudendo esegue l'attach all'oggetto gripper_object
                            if self.closing==True:
                                self.get_logger().info(f"Attach dell'oggetto {self.gripper_object}...")
                    
                                comando = "ros2 service call /attach gazebo_ros_link_attacher/srv/Attach \"{model_name_1: 'tiago', link_name_1: 'gripper_left_finger_link', model_name_2: '"+self.gripper_object+"', link_name_2: 'link'}\" "
                                os.system(comando)

                            # Se si sta aprendo esegue il detch all'oggetto gripper_object
                            else:
                                self.get_logger().info(f"Detach dell'oggetto {self.gripper_object}...")

                                comando = "ros2 service call /detach gazebo_ros_link_attacher/srv/Attach \"{model_name_1: 'tiago', link_name_1: 'gripper_left_finger_link', model_name_2: '"+self.gripper_object+"', link_name_2: 'link'}\" "
                                os.system(comando)

                            time.sleep(2)

                    # Invia un messaggio di termine del movimento su sm_ik_communication
                    self.msg_state=String()
                    self.msg_state.data='terminato'
                    self.sm_ik_publisher.publish(self.msg_state)


                # Esegue le azioni per il gripper
                elif current_task_splitted[0]=='torso':

                    self.get_logger().info(f"Esecuzione del task {self.current_task+1} di {len(self.execution_list)} iniziata: movimento del torso in corso...")

                    # Divide la stringa relativa al task per verificare se il movimento del torso è relativo o assoluto e
                    # calcola la posa da pubblicare
                    relative_torso=self.execution_list[self.current_task].split('rel:')

                    # Movimento assoluto
                    if len(relative_torso)==1:

                        self.torso_pose=float(current_task_splitted[1])

                    # Movimento relativo
                    elif len(relative_torso)==2:
                        
                        self.torso_pose=float(self.current_torso_pose[0])+float(relative_torso[1])

                    # Pubblica la posa per il torso e attende il termine dell'esecuzione

                    self.torso_msg = JointTrajectory()
                    self.torso_msg.joint_names = ['torso_lift_joint']

                    self.torso_point = JointTrajectoryPoint()
                    self.torso_point.positions = [self.torso_pose]
                    self.torso_point.time_from_start.sec = 1

                    self.torso_msg.points.append(self.torso_point)

                    self.publisher_torso.publish(self.torso_msg)

                    time.sleep(3)

                    # Invia un messaggio di termine del movimento su sm_ik_communication
                    self.msg_state=String()
                    self.msg_state.data='terminato'
                    self.sm_ik_publisher.publish(self.msg_state)


                # Al termine del task, aggiorna current_task per permettere l'esecuzione del task successivo
                self.current_task+=1

            else:
                self.get_logger().info(f"Esecuzione dei task terminata.")
                        


    
    def decode_msg(self):

        #La funzione estrae la posa finale da un elemento stringa di execution_list

        
        # Elimina la parte di posa relativa al numero di movimenti da compiere (utile all'inversione cinematica)
        point=self.execution_list[self.current_task].split('_num:')[0]  # pose=1_offset:x=0,y=-0.1,z=-0.1_rot:Rz=-0.5,Rx=0.5

        # Se la posa è relax genera un messaggio vuoto; la posizione verrà gestita dall'inversione cinematica
        if point=='relax':
            pose=PoseStamped()
        else:
            pose=PoseStamped()

            # Divide posa, offset ed eventuali rotazioni
            elements=point.split('_')   #   pose=1    offset:x=0,y=-0.1,z=-0.1    rot:Rz=-0.5,Rx=0.5

            # Esegue una copia della posa i-esima dall'array poses sulla variabile pose
            pose=copy.deepcopy(self.poses[int(elements[0].split('=')[1])])
            
            if len(elements)>1:

                elements.pop(0)
                
                # Esegue l'elaborazione per gli offset e le rotazioni specificate
                for action in elements:
                    
                    # Divide la tipologia di azione dai valori
                    values=action.split(':')    #   offset    x=0,y=-0.1,z=-0.1

                    if values[0]=='offset':

                        # Separa i valori sui diversi assi e aggiorna la posa per ognuno di essi
                        offsets=values[1].split(',')    #   x=0    y=-0.1    z=-0.1
                        
                        for offset in offsets:

                            direction=offset.split('=')[0]  #   x
                            value=float(offset.split('=')[1])  #   0

                            if direction=='x':
                                pose.pose.position.x+=value
                            elif direction=='y':
                                pose.pose.position.y+=value
                            elif direction=='z':
                                pose.pose.position.z+=value

                    elif values[0]=='rot':
                        
                        # Separa i valori sui diversi assi e aggiorna la posa per ognuno di essi
                        rotations=values[1].split(',')  #   Rz=-0.5    Rx=0.5

                        quat_pose=np.array([pose.pose.orientation.x,
                                    pose.pose.orientation.y,
                                    pose.pose.orientation.z,
                                    pose.pose.orientation.w])

                        # Calcola la matrice di rotazione dal quaternione della posa
                        mat_rot_pose=Rotation.from_quat(quat_pose).as_matrix()

                        for rot in rotations:
                            
                            # Separa la direzione di rotazione dal valore moltiplicativo di pigreco
                            rot_splitted=rot.split('=')
                            direction=rot_splitted[0]
                            value=float(rot_splitted[1])

                            if direction=='Rx':
                                mat_rot=np.array(SE3.Rx(value*np.pi))[:3,:3]
                            elif direction=='Ry':
                                mat_rot=np.array(SE3.Ry(value*np.pi))[:3,:3]
                            elif direction=='Rz':
                                mat_rot=np.array(SE3.Rz(value*np.pi))[:3,:3]

                            # Aggiorna la matrice di rotazione
                            mat_rot_pose=np.dot(mat_rot_pose,mat_rot)

                        # Al termine, converte la matrice di rotazione in quaternione e aggiorna la posa
                        quat_pose=Rotation.from_matrix(mat_rot_pose).as_quat()

                        pose.pose.orientation.x=quat_pose[0]
                        pose.pose.orientation.y=quat_pose[1]
                        pose.pose.orientation.z=quat_pose[2]
                        pose.pose.orientation.w=quat_pose[3]
            
        # Restituisce la posa da raggiungere come PoseStamped
        return pose


        
def main():

    rclpy.init()
    node = StateMachine()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()