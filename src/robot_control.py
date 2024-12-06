#!/usr/bin/env python

import rospy

from geometry_msgs.msg import Point, Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2

# Parámetros de control
CENTER_TOLERANCE_X = 100  # Tolerancia en píxeles para el eje X
TARGET_DISTANCE = 1.5  # Distancia deseada en metros
LINEAR_GAIN = 0.5  # Ganancia para el control de la velocidad lineal
ANGULAR_GAIN = 0.005  # Ganancia para el control de la velocidad angular
MAX_VSPEED = 1
MAX_WSPEED = 0.3

class RobotControl:
    def __init__(self):
        # Publicador para el tópico de velocidad
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel_mux/input/navi", Twist, queue_size=10)
        # Suscriptor al tópico /person_pose
        self.person_sub = rospy.Subscriber("/person_pose", Point, self.person_pose_callback)

        # VARIABLES
        self.last_error = 1

    def person_pose_callback(self, msg):
        """Callback para recibir la posición del pixel desde /person_pose."""

        error_x = msg.x
        pixel_depth = msg.y

        if error_x == -1 and pixel_depth == -1:   # Person detection has not detected a person
            twist = Twist()
            twist.linear.x = 0

            if self.last_error > 0:
                twist.angular.z = -MAX_WSPEED 
            elif self.last_error < 0:
                twist.angular.z = MAX_WSPEED 

            
            rospy.loginfo(f"IDLE MOV: Speed: Linear X: {twist.linear.x:.2f}, Angular Z: {twist.angular.z:.2f}")
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
                distance_error = pixel_depth - TARGET_DISTANCE  # Error respecto a la distancia deseada
                last_error = distance_error
                # Control lineal para mantener la distancia
                twist.linear.x = LINEAR_GAIN * distance_error
                
                twist.angular.z = twist.angular.z * distance_error
                if twist.linear.x > MAX_VSPEED:
                    twist.linear.x = MAX_VSPEED
                elif twist.linear.x < -MAX_VSPEED:
                    twist.linear.x = -MAX_VSPEED

                rospy.loginfo(f"Speed: Linear X: {twist.linear.x:.2f}, Angular Z: {twist.angular.z:.2f}")
            else: 
                twist.linear.x = 0
                rospy.loginfo(f"Speed: Linear X: {twist.linear.x:.2f}, Angular Z: {twist.angular.z:.2f}")

            self.cmd_vel_pub.publish(twist)

    
rospy.init_node("Robot_Control")
rospy.loginfo("Robot Control node started. Waiting...")
rc = RobotControl()
rospy.spin()


