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
from mediapipe.tasks import python
from mediapipe.tasks.python import vision

import numpy as np
import math
from message_filters import ApproximateTimeSynchronizer, Subscriber
import time
from main import TOPIC_PERSONPOSE, TOPIC_COMMAND, START_DETECTION_CMD, STOP_DETECTION_CMD, TOPIC_RGBCAM, TOPIC_DEPTHCAM

class BlazePoseDetector:
    def __init__(self):
        # Configuración del modelo MediaPipe Pose Landmarker
        base_options = python.BaseOptions(
            model_asset_path="../tf_model/pose_landmarker_heavy.task",
            delegate=python.BaseOptions.Delegate.GPU
        )
        self.landmarker_options = vision.PoseLandmarkerOptions(
            base_options=base_options,
            running_mode=vision.RunningMode.LIVE_STREAM, # Usamos modo para imágenes
            num_poses = 1,
            min_pose_detection_confidence = 0.1,
            min_pose_presence_confidence = 0.1,
            min_tracking_confidence = 0.1,
            result_callback=self.on_pose_detected
        )
        self.landmarker = vision.PoseLandmarker.create_from_options(self.landmarker_options)

        self.bridge = cv_bridge.CvBridge()
        self.is_active = False
        self.cmd = None
        self.image_sub = None   
        self.depth_sub = None   
        self.cmd_pub = rospy.Publisher(TOPIC_PERSONPOSE, Point, queue_size=10)
        self.simulation = False
        self.ms = 0

        self.frame_rgb = None
        rospy.Subscriber(TOPIC_COMMAND, String, self.handler_callback)
        rospy.loginfo("Person detection waiting...")

        
    def handler_callback(self, msg):
        if msg.data == START_DETECTION_CMD and not self.is_active:
            self.is_active = True
            self.image_sub = rospy.Subscriber(TOPIC_RGBCAM, Image, self.callback)
            if self.simulation:
                self.cap = cv2.VideoCapture(0)
            #self.depth_sub = Subscriber(TOPIC_DEPTHCAM, Image)

            #ats = ApproximateTimeSynchronizer([self.image_sub, self.depth_sub], queue_size=10, slop=0.05)
            #ats.registerCallback(self.callback)
            rospy.loginfo("PERSON DETECTION: Visual control ON")

        elif msg.data == STOP_DETECTION_CMD and self.is_active:
            self.is_active = False
            self.image_sub.unregister()
            self.depth_sub.unregister()
            rospy.loginfo("PERSON DETECTION: Visual control OFF")

    def callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        frame = self.preprocess_depth_image(frame)

        if self.simulation: 
            # Asegúrate de que la cámara se haya inicializado correctamente
            if not self.cap.isOpened():
                rospy.logerr("Error: Unable to open camera")
                return
            ret, frame = self.cap.read()
            
            if not ret:
                rospy.logerr("Error: Unable to read camera")
                return
        
        #depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        self.detect_person(frame)

    def detect_person(self, frame):

         # Convierte la imagen a RGB (MediaPipe espera imágenes en RGB)
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # Procesa el cuadro con `landmarker.detect()`
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=frame_rgb)
        
        self.image_width = frame.shape[1]
        if not self.live_stream:
            result = self.landmarker.detect(mp_image)

            if result.pose_landmarks:
                # Obtén las coordenadas de la cadera derecha (RIGHT_HIP)
                right_hip = result.pose_landmarks[0][24] # Índice 24: RIGHT_HIP
                print(right_hip)
                x, y = int(right_hip.x * frame.shape[1]), int(right_hip.y * frame.shape[0])

                # Dibuja el punto en la imagen
                cv2.circle(frame, (x, y), 5, (0, 255, 0), -1)
        else:
            result = self.landmarker.detect_async(mp_image)
        # Muestra la imagen con el punto de la cadera
        cv2.imshow("Pose Detection", frame)
        cv2.waitKey(1)

    def on_pose_detected(self, result, frame, timestamp):
        

        # Acceder al primer conjunto de landmarks
        if result.pose_landmarks:
            # Obtén las coordenadas de la cadera derecha (RIGHT_HIP)
            right_hip = result.pose_landmarks[0][24] # Índice 24: RIGHT_HIP
            

            # Dibuja el punto en la imagen
            cv2.circle(self.frame_rgb, (right_hip.x, right_hip.y), 5, (0, 255, 0), -1)

            # Publicar la posición como mensaje ROS
            self.cmd = Point()
            self.cmd.x = int(right_hip * self.image_width) - int(self.image_width / 2)
            self.cmd.y = right_hip.z  # Opcional: profundidad si está disponible
            self.cmd_pub.publish(self.cmd)

            rospy.loginfo(f"Person detected at: {self.cmd}")

        else:
            rospy.loginfo(f"No person detected")



rospy.init_node('Person_Detector')
cd  = BlazePoseDetector()
rospy.spin()       
