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

#  rostopic pub /robot_cmd std_msgs/String "start_detection"

""" ******************************************************************************************************
    Definicion de macros y variables globales
"""
TOPIC_VEL = "/cmd_vel_mux/input/navi"
TOPIC_SCAN = '/scan'
TOPIC_COMMAND = '/robot_cmd'
TOPIC_DETECTOR = '/detection_control'
TOPIC_IMAGE = '/image'
TOPIC_PERSONPOSE = '/person_pose'
TOPIC_RGBCAM = '/camera/rgb/image_raw'
TOPIC_DEPTHCAM = '/camera/depth/image_raw'
TOPIC_LOGS = '/robot_logs'

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

# Parámetros de control
CENTER_TOLERANCE_X = 100  # Tolerancia en píxeles para el eje X
TARGET_DISTANCE = 1.5  # Distancia deseada en metros
LINEAR_GAIN = 0.5  # Ganancia para el control de la velocidad lineal
ANGULAR_GAIN = 0.001  # Ganancia para el control de la velocidad angular
MAX_VSPEED = 0.2
MAX_WSPEED = 0.3

states = [IDLE_ST, FOLLOW_ST, MOVE_ST, SHUTDOWN_ST]
rooms = r"^(.*)\b(cocina|baño|sala|habitación|estación)\b$"

# Definir los waypoints a los que el robot debe moverse
waypoints = [
    ['habitación', (-0.3, 4.4), (0.0, 0.0, 0.0, 1.0)],  # Nombre, posición, orientación
    ['estación', (-2.06, 5.82), (0.0, 0.0, 0.0, 1.0)],  # Ejemplo con otro punto
    ['baño', (-3.6, 4.6), (0.0, 0.0, 0, 1.0)],
    ['salón', (-3.6, 0.7), (0.0, 0.0, 0, 1.0)],
    ['cocina', (0.0, 1.6), (0.0, 0.0, 0, 1.0)],
]


""" ******************************************************************************************************
   Clase para el movimiento a un waypoint.
"""  
class MoveState(State):
    def __init__(self):
        State.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'], input_keys=['input_data'])
        self.clientAvailable = True
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        # Espera un máximo de 10 segundos para que el servidor esté disponible
        if not self.client.wait_for_server(timeout=rospy.Duration(2)):
            self.clientAvailable = False
            rospy.logerr("El servidor 'move_base' no está disponible.")
        
            

    def execute(self, userdata):
        # Obtener el waypoint a mover
        
        waypoint_name = userdata.input_data
        waypoint = None
        waypoint = next((w for w in waypoints if w[0] == waypoint_name), None)
        
        if waypoint == None:
            return 'aborted'
                       
        #waypoint in enumerate(waypoints)
        
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

            
            self.client.send_goal(goal_pose)
            self.client.wait_for_result()
        
            if self.client.get_state() == actionlib.GoalStatus.SUCCEEDED:
                return 'succeeded'
            else:
                return 'aborted'
        else:
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
        # VARIABLES
        self.last_error = 1
        self.followPerson = True
        self.cmd = None
        self.dynamicDist = 0
        self.last_twist = Twist()
        
    def execute(self, userdata):
        
        self.subCmd = rospy.Subscriber(TOPIC_COMMAND, String , self.cmd_callback)
        # Suscriptor al tópico /person_pose
        self.cmd = None
        self.subPerson = rospy.Subscriber(TOPIC_PERSONPOSE, Point, self.person_pose_callback)
        self.cmd_vision_pub.publish(START_DETECTION_CMD)
        rate = rospy.Rate(0.1)

        while not rospy.is_shutdown() and self.followPerson:
            rate.sleep()

            if self.cmd == STOP_FOLLOW_CMD or self.cmd in self.states or MOVE_ST in self.cmd:
                self.subPerson.unregister()
                self.subCmd.unregister()

                self.dynamicDist = 0
                self.last_error = 0
                self.followPerson = True
                userdata.output_data = None

                if self.cmd in self.states:
                    userdata.output_data = self.cmd

                self.cmd_vision_pub.publish(STOP_DETECTION_CMD)
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

        error_x = msg.x
        pixel_depth = msg.y
        #print("HOLA")
        if error_x == -1 and pixel_depth == -1:   # Person detection has not detected a person
            twist = Twist()
            twist.linear.x = 0

            if self.last_error > 0:
                twist.angular.z = -MAX_WSPEED 
            elif self.last_error < 0:
                twist.angular.z = MAX_WSPEED 
            #rospy.loginfo(f"IDLE MOV: Speed: Linear X: {twist.linear.x:.2f}, Angular Z: {twist.angular.z:.2f}")
            print("Searching...")
            self.cmd_vel_pub.publish(twist)

        # SI HAY UNA PERSONA EN LA IMAGEN
        else:
            # Obtener la profundidad en el pixel correspondiente
            
            twist = Twist()
            # Control angular para corregir el error
            if abs(error_x) > CENTER_TOLERANCE_X and error_x != -1:
                print("error_x: ", error_x)
                twist.angular.z = -ANGULAR_GAIN * error_x
                twist.linear.x = 0

                print("angular: ", twist.angular.z)
                if twist.angular.z > MAX_WSPEED:
                    twist.angular.z = MAX_WSPEED
                elif twist.angular.z < -MAX_WSPEED:
                    twist.angular.z = -MAX_WSPEED
            else:
                # Comprobar si la profundidad es válida
                if pixel_depth != -1:
                    distance_error = pixel_depth - (TARGET_DISTANCE + self.dynamicDist) # Error respecto a la distancia deseada

                    self.last_error = distance_error
                    # Control lineal para mantener la distancia
                    #twist.linear.x = LINEAR_GAIN * distance_error
                    if pixel_depth > TARGET_DISTANCE:
                        twist.linear.x = MAX_VSPEED
                    else:
                        twist.linear.x = 0

                    print("Depth: ", distance_error)

                    twist.angular.z = 0
                    #rospy.loginfo(f"Speed: Linear X: {twist.linear.x:.2f}, Angular Z: {twist.angular.z:.2f}")
                else: 
                    print("Pixel depth error")
                    twist.linear.x = self.last_twist.linear.x
                    twist.angular.z = self.last_twist.angular.z
                    #rospy.loginfo(f"Speed: Linear X: {twist.linear.x:.2f}, Angular Z: {twist.angular.z:.2f}")

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
        self.idleWait = True
        self.place = None

    def cmd_callback(self, msg):
        if ":" in msg.data: 
            cmd, self.place = msg.data.split(":")
        else:
            cmd = msg.data

        if cmd in states:
            self.idleWait = False
            self.cmd = cmd

    def execute(self, userdata):
        rospy.loginfo("IDLE WAIT: Waiting for next state...")  

        # Espera hasta que se reciba un comando válido
        while not rospy.is_shutdown() and self.idleWait:
            rospy.sleep(0.1)  # Espera brevemente antes de verificar nuevamente

        if userdata.input_data in states:
            cmd = userdata.input_data
            userdata.output_data = cmd
            return userdata.input_data
        
        if self.cmd in states:
            userdata.output_data = self.place
            cmd = self.cmd
            self.cmd = None
            return cmd

        self.idleWait = True
        return IDLE_ST
        

""" ******************************************************************************************************
   Funcion principal
"""    

def stop_all_nodes():
    rospy.loginfo("Recuperando lista de nodos...")
    try:
        # Obtener todos los nodos en ejecución
        nodes = rosnode.get_node_names()

        # Separar el nodo "main" del resto
        main_node = None
        other_nodes = []
        for node in nodes:
            if node == "/main":  # Nombre exacto del nodo principal
                main_node = node

            elif node != "/bash_interface":
                other_nodes.append(node)

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

    # Crear un contenedor de concurrencia
    sm = StateMachine(outcomes=['end'])
    sm.userdata.data = None
    with sm:
               
        StateMachine.add('IdleWait', 
            IdleWait(), 
            transitions={
                IDLE_ST:'IdleWait',
                FOLLOW_ST:'FollowPerson',
                MOVE_ST:'MoveState',
                SHUTDOWN_ST:'end'},
            remapping={'input_data':'data',
                       'output_data':'data'})

        StateMachine.add('FollowPerson', 
            FollowPerson(), 
            transitions={
                HANDLE_ST:'IdleWait'},
            remapping={'input_data':'data',
                       'output_data':'data'})
        
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

    rospy.spin()   


if __name__ == '__main__':
    main()