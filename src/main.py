#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import subprocess
import signal

import rospy
import rosnode
import cv2, cv_bridge
import smach_ros
import math
import random
import numpy as np

from smach import State, StateMachine
from smach_ros import SimpleActionState

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist, Point

import actionlib
import re

#  rostopic pub /robot_cmd std_msgs/String "start_detection"

""" ******************************************************************************************************
    Definicion de macros y variables globales
"""
TOPIC_VEL = "/cmd_vel_mux/input/navi"
TOPIC_SCAN = '/scan'
TOPIC_AMCLPOS = '/amcl_pose'
TOPIC_COMMAND = '/robot_cmd'
TOPIC_DETECTOR = '/detection_control'
TOPIC_IMAGE = '/image'
TOPIC_PERSONPOSE = '/person_pose'
TOPIC_RGBCAM = '/camera/rgb/image_raw'
TOPIC_DEPTHCAM = '/camera/depth/image_raw'
TOPIC_LOGS = '/robot_logs'
TOPIC_PRIMG = '/processed_image'

# MENSAJES
MOVE_CLOSER_CMD = "move_closer"
MOVE_AWAY_CMD = "move_away"
RESET_DIST_CMD = "reset_dist"

START_DETECTION_CMD = "start_detection"
STOP_DETECTION_CMD = "stop_detection"

START_FOLLOW_CMD = "start_follow_person"
STOP_FOLLOW_CMD = "stop_follow_person"

START_MOVE_CMD = "start_move_person"
STOP_MOVE_CMD = "stop_move_person"

START_VOICE_CMD = "start_voice_control"
STOP_VOICE_CMD = "stop_voice_control"

SHUTDOWN_CMD = "goodbye"


# ESTADOS
IDLE_ST = "idle_state"
FOLLOW_ST = "follow_state"
MOVE_ST = "move_state"
HANDLE_ST = "handle_state"
SHUTDOWN_ST = "shutdown_state"
BASE_ST = "base_state"

# PATH
WAYPOINT_PATH = "/home/asahel/ROS_WS/src/proyecto_servicios/src/qr_code_log.txt"
# Parámetros de control
CENTER_TOLERANCE_X = 50  # Tolerancia en píxeles para el eje X
TARGET_DISTANCE = 1.5  # Distancia deseada en metros
DISTANCE_ERROR = 0.1
LINEAR_GAIN = 0.5  # Ganancia para el control de la velocidad lineal
ANGULAR_GAIN = 0.001  # Ganancia para el control de la velocidad angular
MAX_VSPEED = 0.5
MAX_WSPEED = 0.5

states = [IDLE_ST, FOLLOW_ST, MOVE_ST, SHUTDOWN_ST]
rooms = r"^(.*)\b(cocina|wc|salon|habitacion|estacion)\b$"

# Definir los waypoints a los que el robot debe moverse
def_waypoints = [
    ['habitacion', (-0.3, 4.4), (0.0, 0.0, 0.0, 1.0)],  # Nombre, posición, orientación
    ['estacion', (-2.06, 5.82), (0.0, 0.0, 0.0, 1.0)],  # Ejemplo con otro punto
    ['wc', (-3.6, 4.6), (0.0, 0.0, 0, 1.0)],
    ['salon', (-3.6, 0.7), (0.0, 0.0, 0, 1.0)],
    ['cocina', (0.0, 1.6), (0.0, 0.0, 0, 1.0)],
]

""" ******************************************************************************************************
   Clase para el movimiento a un waypoint.
"""  
class MoveState(State):
    def __init__(self):
        global states
        self.states = [state for state in states if state != MOVE_ST]
        State.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'], input_keys=['input_data'], output_keys=['output_data'])
        self.clientAvailable = True
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.cmd = None
        self.log_pub = rospy.Publisher(TOPIC_LOGS, String, queue_size=10)  

        self.log_msg = "[INFO] MOVE STATE: Waiting..."

        # Espera un máximo de 10 segundos para que el servidor esté disponible
        if not self.client.wait_for_server(timeout=rospy.Duration(5)):
            self.clientAvailable = False
            rospy.logerr("'move_base' server not available")

    def update_waypoints_from_file(self, filename, waypoints):
        """
        Funcion para leer el fichero de waypoints y actualizar la lista
        """
        with open(filename, 'r') as file:
            lines = file.readlines()

        # Expresión regular para parsear las líneas
        pattern = re.compile(r"^(.*?), \((.*?), (.*?), (.*?)\), \((.*?), (.*?), (.*?), (.*?)\)$")

        for line in lines:
            match = pattern.match(line.strip())
            if match:
                name = match.group(1).strip()
                position = tuple(map(float, match.group(2, 3, 4)))
                orientation = tuple(map(float, match.group(5, 6, 7, 8)))

                # Verificar si el nombre ya está en la lista
                updated = False
                for waypoint in waypoints:
                    if waypoint[0] == name:
                        waypoint[1] = position
                        waypoint[2] = orientation
                        updated = True
                        self.log_pub.publish(f"[INFO] MOVE STATE: Updated waypoint list from {filename}")
                        break

                # Si no se encuentra, añadir uno nuevo
                if not updated:
                    waypoints.append([name, position, orientation])

    def publish_message(self, event):
        # Publish the message
        self.log_pub.publish(self.log_msg)

    def cmd_callback(self, msg):

        if msg.data in self.states:
            self.abort_movement = True
            self.client.cancel_all_goals()  # Cancela el objetivo en move_base
            rospy.loginfo("[INFO] MOVE STATE: Movement aborted by command.")
            self.log_pub.publish("[INFO] MOVE STATE: Movement aborted by command.")
            self.cmd = msg.data

    def execute(self, userdata):
        
        self.log_msg = None
        self.subCmd = rospy.Subscriber(TOPIC_COMMAND, String , self.cmd_callback)
        self.timer = rospy.Timer(rospy.Duration(10), self.publish_message)

        # Obtiene el waypoint deseado
        self.update_waypoints_from_file(WAYPOINT_PATH, def_waypoints)
        waypoint_name = userdata.input_data
        print(def_waypoints)
        waypoint = None
        waypoint = next((w for w in def_waypoints if w[0] == waypoint_name), None)
        
        # Si no existe, se cancela la orden de movimiento
        if waypoint == None:
            self.log_pub.publish(f"[INFO] MOVE STATE: {waypoint_name} not found")
            self.timer.shutdown()
            return 'aborted'
                       
        
        # Si el cliente move_base esta activo
        if self.clientAvailable:
            goal_pose = MoveBaseGoal()
            goal_pose.target_pose.header.frame_id = 'map'
            goal_pose.target_pose.pose.position.x = waypoint[1][0]
            goal_pose.target_pose.pose.position.y = waypoint[1][1]
            goal_pose.target_pose.pose.position.z = 0.0
            goal_pose.target_pose.pose.orientation.x = waypoint[2][0]
            goal_pose.target_pose.pose.orientation.y = waypoint[2][1]
            goal_pose.target_pose.pose.orientation.z = waypoint[2][2]
            goal_pose.target_pose.pose.orientation.w = waypoint[2][3]

            self.log_msg = f"[INFO] MOVE STATE: Moving to {waypoint_name}..."
            self.client.send_goal(goal_pose)
            self.client.wait_for_result()
        
            if self.client.get_state() == actionlib.GoalStatus.SUCCEEDED:
                self.log_pub.publish(f"[INFO] MOVE STATE: {waypoint_name} reached")
                self.log_pub.publish(f"[EVENT]: MOVE STATE -> IDLE STATE")
                self.timer.shutdown()
                return 'succeeded'
            else:
                self.log_pub.publish(f"[INFO] MOVE STATE: {waypoint_name} not found")
                self.log_pub.publish(f"[EVENT]: MOVE STATE -> IDLE STATE")
                self.timer.shutdown()
                userdata.output_data = self.cmd
                return 'aborted'
        else:
            self.log_pub.publish(f"[EVENT]: MOVE STATE -> IDLE STATE")
            self.timer.shutdown()
            return 'aborted'
        
        
""" ******************************************************************************************************
   Clase para el estado de seguimiento de personas.
"""  
class FollowPerson(State):
    def __init__(self):
        global states
        self.states = [state for state in states if state != FOLLOW_ST]
        State.__init__(self, outcomes=[HANDLE_ST], input_keys=['input_data'], output_keys=['output_data'])
        # Publicador para el tópico de velocidad
        self.cmd_vel_pub = rospy.Publisher(TOPIC_VEL, Twist, queue_size=10)
        self.cmd_vision_pub = rospy.Publisher(TOPIC_COMMAND, String, queue_size=10)
        self.log_pub = rospy.Publisher(TOPIC_LOGS, String, queue_size=10)
        
        self.log_msg = "[INFO] FOLLOW STATE: Waiting..."
        # VARIABLES
        self.last_error = 1
        self.followPerson = True
        self.cmd = None
        self.dynamicDist = 0
        self.last_twist = Twist()
        self.last_twist.angular.z = 0
        self.last_twist.linear.x = 0
        
    def publish_message(self, event):
        # Publish the message
        self.log_pub.publish(self.log_msg)

    def execute(self, userdata):
        
        self.subCmd = rospy.Subscriber(TOPIC_COMMAND, String , self.cmd_callback)
        self.timer = rospy.Timer(rospy.Duration(5), self.publish_message)
        # Suscriptor al tópico /person_pose
        self.cmd = None
        self.subPerson = rospy.Subscriber(TOPIC_PERSONPOSE, Point, self.person_pose_callback)
        self.cmd_vision_pub.publish(START_DETECTION_CMD)
        self.log_pub.publish("[INFO] FOLLOW STATE: Started state")
        rate = rospy.Rate(0.1)

        while not rospy.is_shutdown() and self.followPerson:
            rate.sleep()

            if self.cmd == STOP_FOLLOW_CMD or self.cmd in self.states or MOVE_ST in self.cmd:
                self.log_pub.publish("[INFO] FOLLOW STATE: Stopped state")
                self.subPerson.unregister()
                self.subCmd.unregister()

                self.dynamicDist = 0
                self.last_error = 0
                self.followPerson = False
                userdata.output_data = None

                if self.cmd in self.states:
                    userdata.output_data = self.cmd              
                
                self.cmd_vision_pub.publish(STOP_DETECTION_CMD)
                self.log_pub.publish(f"[EVENT]: FOLLOW STATE -> IDLE STATE")
                self.timer.shutdown()
                return HANDLE_ST

    def cmd_callback(self, msg):
   
        if msg.data == MOVE_CLOSER_CMD:
            self.dynamicDist = self.dynamicDist - 0.5
        elif msg.data == MOVE_AWAY_CMD:
            self.dynamicDist = self.dynamicDist + 0.5
        elif msg.data == RESET_DIST_CMD:
            self.dynamicDist = 0

        self.cmd = msg.data

    def person_pose_callback(self, msg):
        """Callback para recibir la posición del pixel desde /person_pose."""
        if self.cmd == STOP_FOLLOW_CMD or self.cmd in self.states or MOVE_ST in self.cmd:
            self.subPerson.unregister()
            
        
        error_x = msg.x
        pixel_depth = msg.y
        
        if error_x == -1 and pixel_depth == -1:   # Person detection has not detected a person
            twist = Twist()
            twist.linear.x = 0

            if self.last_error > 0:
                twist.angular.z = -MAX_WSPEED 
            else:
                twist.angular.z = MAX_WSPEED 
            
            self.log_msg = "[INFO] FOLLOW STATE: Searching for person..."
            self.cmd_vel_pub.publish(twist)

        # SI HAY UNA PERSONA EN LA IMAGEN
        else:
            # Obtener la profundidad en el pixel correspondiente
            self.log_msg = "[INFO] FOLLOW STATE: Person found. Following..."
            twist = Twist()
            # Control angular para corregir el error
            if abs(error_x) > CENTER_TOLERANCE_X and error_x != -1:
                
                twist.angular.z = -ANGULAR_GAIN * error_x

                if twist.angular.z > MAX_WSPEED:
                    twist.angular.z = MAX_WSPEED
                elif twist.angular.z < -MAX_WSPEED:
                    twist.angular.z = -MAX_WSPEED

                self.last_twist.angular.z = twist.angular.z

            if pixel_depth != -1:
                distance_error = pixel_depth - (TARGET_DISTANCE + self.dynamicDist) # Error respecto a la distancia deseada

                self.last_error = distance_error
                # Control lineal para mantener la distancia
                twist.linear.x = LINEAR_GAIN * distance_error

                if(abs(distance_error) < DISTANCE_ERROR):
                    twist.linear.x = 0

                if twist.linear.x  > MAX_VSPEED:
                    twist.linear.x  = MAX_VSPEED
                elif twist.linear.x  < -MAX_VSPEED:
                    twist.linear.x  = -MAX_VSPEED

            else: 
                twist.linear.x = self.last_twist.linear.x
                twist.angular.z = self.last_twist.angular.z

            self.last_twist.linear.x = twist.linear.x
            self.last_twist.angular.z = twist.angular.z

            self.cmd_vel_pub.publish(twist)


""" ******************************************************************************************************
   Para esuchar constantemente el TOPIC de CMD
""" 

class IdleWait(State):
    def __init__(self):
        global states
        State.__init__(self, outcomes=states, input_keys=['input_data'], output_keys=['output_data'])
        self.cmd = None  # Variable para almacenar el último comando recibido
        self.subCmd = rospy.Subscriber(TOPIC_COMMAND, String, self.cmd_callback)
        self.log_pub = rospy.Publisher(TOPIC_LOGS, String, queue_size=10)

        self.idleWait = True
        self.place = None
        self.log_msg = "[INFO] IDLE STATE: Waiting for next state..."

    def publish_message(self, event):
        # Publish the message
        self.log_pub.publish(self.log_msg)

    def execute(self, userdata):
        rospy.loginfo("[INFO] IDLE STATE: Waiting for next state...")  
        self.timer = rospy.Timer(rospy.Duration(10), self.publish_message)

        # Espera hasta que se reciba un comando válido
        while not rospy.is_shutdown() and self.idleWait:
            rospy.sleep(0.1)  # Espera brevemente antes de verificar nuevamente

        if userdata.input_data in states:
            cmd = userdata.input_data
            userdata.output_data = cmd
            self.log_pub.publish(f"[EVENT]: IDLE STATE -> {cmd}")
            self.timer.shutdown()
            return userdata.input_data
        
        if self.cmd in states:
            userdata.output_data = self.place
            self.log_pub.publish(f"[EVENT]: IDLE STATE -> {self.cmd}")
            self.timer.shutdown()
            cmd = self.cmd
            self.cmd = None   
            return cmd

        self.idleWait = True
        self.timer.shutdown()
        return IDLE_ST
                    
    def cmd_callback(self, msg):
        if ":" in msg.data: 
            cmd, self.place = msg.data.split(":")
        else:
            cmd = msg.data

        if cmd in states:
            self.idleWait = False
            self.cmd = cmd

""" ******************************************************************************************************
   Funcion principal
"""    

def stop_all_nodes():
    rospy.loginfo("Recuperando lista de nodos...")
    try:
        # Obtener todos los nodos en ejecución
        nodes = rosnode.get_node_names()
        log_pub = rospy.Publisher(TOPIC_LOGS, String, queue_size=10)
        # Separar el nodo "main" del resto
        main_node = None
        other_nodes = []
        for node in nodes:
            if node == "/main":  # Nombre exacto del nodo principal
                main_node = node

            elif node != "/bash_interface" or node != "/visual_interface" :
                other_nodes.append(node)
        
        log_pub.publish(f"[EVENT]: CLOSING APP")

        # Apagar todos los nodos excepto "main"
        for node in other_nodes:
            try:
                rospy.loginfo(f"Apagando nodo: {node}")
                os.system(f"rosnode kill {node}")
            except Exception as e:
                rospy.logwarn(f"Error al intentar apagar {node}: {e}")

        # Finalmente, apagar el nodo "main"
        if main_node:
            try:
                rospy.loginfo(f"Apagando el nodo principal: {main_node}")
                os.system(f"rosnode kill {main_node}")
                rospy.signal_shutdown("Apagando")
                
            except Exception as e:
                rospy.logwarn(f"Error al intentar apagar {main_node}: {e}")

    except Exception as e:
        rospy.logerr(f"Error al recuperar los nodos: {e}")
        
def kill_run_launch():
    try:
        # Ejecuta el comando para buscar procesos relacionados con run.launch
        result = subprocess.check_output(
            "ps aux | grep '[r]oslaunch.*run.launch'",
            shell=True,
            text=True
        )
        
        # Filtrar el PID del resultado
        lines = result.strip().split("\n")
        for line in lines:
            parts = line.split()
            pid = int(parts[1])  # El PID es la segunda columna
            print(f"Encontrado proceso run.launch con PID: {pid}")
            
            # Termina el proceso
            os.kill(pid, signal.SIGTERM)  # O usa signal.SIGKILL para forzar
            print(f"Proceso con PID {pid} terminado correctamente.")
            
    except subprocess.CalledProcessError:
        print("No se encontró ningún proceso relacionado con run.launch.")
    except Exception as e:
        print(f"Error al intentar matar el proceso: {e}")

def main():
    rospy.init_node("main")
    log_pub = rospy.Publisher(TOPIC_LOGS, String, queue_size=10)
    # Crea una maquina de estados mediante SMACH
    sm = StateMachine(outcomes=['end'])
    
    log_pub.publish("[EVENT]: STARTING APP")
    sm.userdata.data = None

    with sm:
        # Estado IdleWait, el robot espera a recibir un comando               
        StateMachine.add('IdleWait', 
            IdleWait(), 
            transitions={
                IDLE_ST:'IdleWait',
                FOLLOW_ST:'FollowPerson',
                MOVE_ST:'MoveState',
                SHUTDOWN_ST:'end'},
            remapping={'input_data':'data',
                       'output_data':'data'})

        # Estado FollowPerson, el robot sigue a una persona
        StateMachine.add('FollowPerson', 
            FollowPerson(), 
            transitions={
                HANDLE_ST:'IdleWait'},
            remapping={'input_data':'data',
                       'output_data':'data'})
        
        # Estado MoveState, el robot se mueve hacia una posicion
        StateMachine.add('MoveState', 
            MoveState(), 
            transitions={
                'succeeded': 'IdleWait', 
                'aborted': 'IdleWait',
                'preempted': 'IdleWait'}, 
            remapping={'input_data': 'data'})
        
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()
    res = sm.execute()

    if res == 'end':
        stop_all_nodes()
        kill_run_launch()
        rospy.signal_shutdown('exit')

    rospy.spin()   


if __name__ == '__main__':
    main()