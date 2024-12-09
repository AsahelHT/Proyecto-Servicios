#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2, cv_bridge
import smach_ros
import math
import random
import numpy as np

from smach import State,StateMachine, Concurrence
from smach_ros import SimpleActionState

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist, Point

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

# ESTADOS
IDLE_ST = "idle_state"
FOLLOW_ST = "follow_state"
MOVE_ST = "move_state"
HANDLE_ST = "handle_state"
     
# Parámetros de control
CENTER_TOLERANCE_X = 100  # Tolerancia en píxeles para el eje X
TARGET_DISTANCE = 1.5  # Distancia deseada en metros
LINEAR_GAIN = 0.5  # Ganancia para el control de la velocidad lineal
ANGULAR_GAIN = 0.005  # Ganancia para el control de la velocidad angular
MAX_VSPEED = 1
MAX_WSPEED = 0.3

states = [  IDLE_ST,
            FOLLOW_ST]

""" ******************************************************************************************************
   Clase para el estado de seguimiento de personas.
"""  
class FollowPerson(State):
    def __init__(self):
        global states
        self.states = [state for state in states if state != FOLLOW_ST]
        State.__init__(self, outcomes=[HANDLE_ST, FOLLOW_ST],output_keys=['output_data'])
        # Publicador para el tópico de velocidad
        self.cmd_vel_pub = rospy.Publisher(TOPIC_VEL, Twist, queue_size=10)
        self.cmd_vision_pub = rospy.Publisher(TOPIC_COMMAND, String, queue_size=10)
        # VARIABLES
        self.last_error = 1
        self.followPerson = True
        self.cmd = None
        self.dynamicDist = 0

    def execute(self, userdata):
        
        self.subCmd = rospy.Subscriber(TOPIC_COMMAND, String , self.cmd_callback)
        # Suscriptor al tópico /person_pose
        self.cmd = None
        self.subPerson = rospy.Subscriber(TOPIC_PERSONPOSE, Point, self.person_pose_callback)
        self.cmd_vision_pub.publish(START_DETECTION_CMD)
        rate = rospy.Rate(0.1)

        while not rospy.is_shutdown() and self.followPerson:
            rate.sleep()

        if not self.followPerson:
            self.subPerson.unregister()
            self.subCmd.unregister()

        if self.cmd == STOP_FOLLOW_CMD:
            return FOLLOW_ST
        elif self.cmd in self.states:
            userdata.output_data = self.cmd
            self.cmd_vision_pub.publish(STOP_DETECTION_CMD)
            return HANDLE_ST

    def cmd_callback(self, msg):
        if msg.data in self.states or msg.data == STOP_FOLLOW_CMD:
            self.followPerson = False
    
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
            self.cmd_vel_pub.publish(twist)

        # Verificar que el pixel esté dentro de los límites de la imagen
        else:
            # Obtener la profundidad en el pixel correspondiente
            
            twist = Twist()
            # Control angular para corregir el error
            if abs(error_x) > CENTER_TOLERANCE_X and error_x != -1:
                twist.angular.z = -ANGULAR_GAIN * error_x

                if twist.angular.z > MAX_WSPEED:
                    twist.angular.z = MAX_WSPEED
                elif twist.angular.z < -MAX_WSPEED:
                    twist.angular.z = -MAX_WSPEED
            else:
                twist.angular.z = 0.0

            # Comprobar si la profundidad es válida
            if pixel_depth != -1:
                distance_error = pixel_depth - (TARGET_DISTANCE + self.dynamicDist) # Error respecto a la distancia deseada
                self.last_error = distance_error
                # Control lineal para mantener la distancia
                twist.linear.x = LINEAR_GAIN * distance_error
                
                twist.angular.z = twist.angular.z * distance_error
                if twist.linear.x > MAX_VSPEED:
                    twist.linear.x = MAX_VSPEED
                elif twist.linear.x < -MAX_VSPEED:
                    twist.linear.x = -MAX_VSPEED
                #rospy.loginfo(f"Speed: Linear X: {twist.linear.x:.2f}, Angular Z: {twist.angular.z:.2f}")
            else: 
                twist.linear.x = 0
                #rospy.loginfo(f"Speed: Linear X: {twist.linear.x:.2f}, Angular Z: {twist.angular.z:.2f}")

            self.cmd_vel_pub.publish(twist)



""" ******************************************************************************************************
   Para esuchar constantemente el TOPIC de CMD
""" 

class IdleWait(State):
    def __init__(self):
        global states
        State.__init__(self, outcomes=states, input_keys=['input_data'])
        self.cmd = None  # Variable para almacenar el último comando recibido
        self.subCmd = rospy.Subscriber(TOPIC_COMMAND, String, self.cmd_callback)
        self.idleWait = True

    def cmd_callback(self, msg):
        if msg.data in states:
            self.idleWait = False
            self.cmd = msg.data

    def execute(self, userdata):
        rospy.loginfo("IDLE WAIT: Waiting for next state...")
        
        if userdata.input_data in states:
            return userdata.input_data

        # Espera hasta que se reciba un comando válido
        while not rospy.is_shutdown() and self.idleWait:
            rospy.sleep(0.1)  # Espera brevemente antes de verificar nuevamente

        if self.cmd in states:
            return self.cmd

""" ******************************************************************************************************
   Funcion principal
"""    

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
                FOLLOW_ST:'FollowPerson'},
            remapping={'input_data':'data'})

        StateMachine.add('FollowPerson', 
            FollowPerson(), 
            transitions={
                HANDLE_ST:'IdleWait',
                FOLLOW_ST:'FollowPerson'},
            remapping={'output_data':'data'})
        

    #sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    #sis.start()
    sm.execute()
    rospy.spin()   


if __name__ == '__main__':
    main()