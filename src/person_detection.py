#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Image
import cv_bridge
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import String, Int32
from std_msgs.msg import Bool
import cv2
import mediapipe as mp
import numpy as np
import math
from message_filters import ApproximateTimeSynchronizer, Subscriber

class BlazePoseDetector:
    def __init__(self):
        # Inicializar MediaPipe Pose
        self.mp_pose = mp.solutions.pose
        self.pose = self.mp_pose.Pose(static_image_mode=False,  # Para detección en tiempo real
                                      model_complexity=0,       # Modelo estándar
                                      min_detection_confidence=0.2,
                                      min_tracking_confidence=0.2)
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_drawing_styles = mp.solutions.drawing_styles

        self.bridge = cv_bridge.CvBridge()
        self.is_active = False
        self.simulation = False
        self.cmd = None

        self.image_sub = None   
        self.depth_sub = None   
        self.cmd_pub = rospy.Publisher('/person_pose', Point, queue_size=1)

        rospy.Subscriber('/robot_command', String, self.handler_callback)
        
        rospy.loginfo("Person detection waiting...")

    def handler_callback(self, msg):
        """
            THIS METHOD HANDLES WHENEVER THIS NODE MUST BE SUSCRIBED TO IMAGE TOPICS OR NOT
            AND HANDLES THE VISUAL CONTROL 
        """
        if msg.data == "start_follow" and not self.is_active:
            self.is_active = True

            if self.simulation:
                self.cap = cv2.VideoCapture(0)

            self.image_sub = Subscriber('/camera/rgb/image_raw', Image)
            self.depth_sub = Subscriber("/camera/depth/image_raw", Image)

            # Sincronización aproximada de mensajes
            ats = ApproximateTimeSynchronizer([self.image_sub, self.depth_sub], queue_size=1, slop=0.5)
            ats.registerCallback(self.callback)
            rospy.loginfo("Visual control ON")

        elif msg.data == "stop_follow" and self.is_active:
            self.is_active = False
            self.image_sub.unregister()
            self.depth_sub.unregister()
            if self.cap:
                self.cap.release()
            cv2.destroyAllWindows()
            rospy.loginfo("Visual control OFF")

    def callback(self, rgb_msg, depth_msg):
        rgb_image = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")

        # Convertir la imagen de profundidad
        depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        
        self.detect_person(rgb_image, depth_image)
        
    def detect_person(self, image, depth_image):
        # Convertir la imagen a RGB para MediaPipe
        rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        #img = depth_image.copy()
        
        # Procesar la imagen con BlazePose
        results = self.pose.process(rgb_image)

        # Verificar si hay landmarks detectados
        if results.pose_landmarks:
            landmarks = results.pose_landmarks.landmark

            # Obtener coordenadas normalizadas de las caderas
            left_hip = landmarks[self.mp_pose.PoseLandmark.LEFT_HIP]
            right_hip = landmarks[self.mp_pose.PoseLandmark.RIGHT_HIP]

            # Calcular el centro entre las caderas (en coordenadas normalizadas)
            center_x = (left_hip.x + right_hip.x) / 2
            center_y = (left_hip.y + right_hip.y) / 2

            # Convertir las coordenadas normalizadas a píxeles
            image_height, image_width, _ = image.shape
            pixel_x = int(center_x * image_width)
            pixel_y = int(center_y * image_height)

            error_x = pixel_x - int(image_width / 2)  # Error respecto al centro de la imagen
            self.cmd = Point()
            self.cmd.x = error_x
            pixel_depth = 0
            if 0 <= pixel_x < image_width and 0 <= pixel_y < image_height:
                depth = depth_image[pixel_y, pixel_x]
                pixel_depth = round(depth, 1)
                

                if np.isfinite(pixel_depth):
                    self.cmd.y = pixel_depth
                else:
                    rospy.logwarn("Invalid pixel depth at: ({pixel_x}, {pixel_y})")
                    self.cmd.y = -1
            else:
                self.cmd.y = -1
                
            # Dibujar el punto central en la imagen
            
            #cv2.circle(img, (pixel_x, pixel_y), 5, (0, 0, 255), -1)

            rospy.loginfo(f"Person detected in: ({pixel_x}, {pixel_y}) at {pixel_depth} meters")
            
            # Dibujar puntos clave en la imagen
            
            """
            self.mp_drawing.draw_landmarks(
                image,
                results.pose_landmarks,
                self.mp_pose.POSE_CONNECTIONS,
                landmark_drawing_spec=self.mp_drawing_styles.get_default_pose_landmarks_style()
            )
            """
        else:
            self.cmd = Point()
            self.cmd.x = -1
            self.cmd.y = -1
            rospy.logwarn("NO PERSON DETECTED")
        
        rospy.loginfo(f"Publishing: Error: {self.cmd.x}, Distance: {self.cmd.y}")
            
        self.cmd_pub.publish(self.cmd)
        #cv2.imshow('Person Detection', depth_image)
        #cv2.waitKey(1)
        return image


rospy.init_node('Person_Detector')
cd  = BlazePoseDetector()
rospy.spin()       
