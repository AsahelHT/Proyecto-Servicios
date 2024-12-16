#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Image
import cv_bridge
from geometry_msgs.msg import Point
from std_msgs.msg import String
import cv2
import numpy as np
import mediapipe as mp

from message_filters import ApproximateTimeSynchronizer, Subscriber

from main import TOPIC_PERSONPOSE, TOPIC_COMMAND, START_DETECTION_CMD, STOP_DETECTION_CMD, TOPIC_RGBCAM, TOPIC_DEPTHCAM, TOPIC_LOGS

EDGES = {
    (0, 1): 'm', (0, 2): 'c', (1, 3): 'm', (2, 4): 'c', (0, 5): 'm', (0, 6): 'c',
    (5, 7): 'm', (7, 9): 'm', (6, 8): 'c', (8, 10): 'c', (5, 6): 'y', (5, 11): 'm',
    (6, 12): 'c', (11, 12): 'y', (11, 13): 'm', (13, 15): 'm', (12, 14): 'c', (14, 16): 'c'
}

class MoveNetDetector:
    def __init__(self):
        # Cargar modelo MoveNet
        self.mp_pose = mp.solutions.pose
        self.pose = self.mp_pose.Pose(static_image_mode=False,  # Para detección en tiempo real
                                      model_complexity=0,       # Modelo estándar
                                      min_detection_confidence=0.3,
                                      min_tracking_confidence=0.3)

        self.bridge = cv_bridge.CvBridge()
        self.is_active = False
        self.cmd = None
        self.image_sub = None   
        self.depth_sub = None   
        self.show_img = False
        self.cmd_pub = rospy.Publisher(TOPIC_PERSONPOSE, Point, queue_size=10)
        self.log_pub = rospy.Publisher(TOPIC_LOGS, String, queue_size=10)
        rospy.Subscriber(TOPIC_COMMAND, String, self.handler_callback)
        rospy.loginfo("Person detection waiting...")

    def handler_callback(self, msg):
        """
        Maneja cuándo este nodo debe suscribirse a los temas de imagen o no.
        """
        if msg.data == START_DETECTION_CMD and not self.is_active:
            self.is_active = True
            self.image_sub = Subscriber(TOPIC_RGBCAM, Image)
            self.depth_sub = Subscriber(TOPIC_DEPTHCAM, Image)
            ats = ApproximateTimeSynchronizer([self.image_sub, self.depth_sub], queue_size=10, slop=0.05)
            ats.registerCallback(self.callback)
            rospy.loginfo("PERSON DETECTION: Camera ON")
            self.log_pub.publish("[VISION]: PERSON DETECTION: Camera ON")

        elif msg.data == STOP_DETECTION_CMD and self.is_active:
            self.is_active = False
            self.image_sub.unregister()
            self.depth_sub.unregister()
            rospy.loginfo("PERSON DETECTION: Camera OFF")
            self.log_pub.publish("[VISION]: PERSON DETECTION: Camera OFF")

    def callback(self, rgb_msg, depth_msg):
        rgb_image = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
        depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        self.detect_person(rgb_image, depth_image)

    def detect_person(self, image, depth_image):
        
        rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        results = self.pose.process(rgb_image)

        self.cmd = Point()
        self.cmd.x = -1
        self.cmd.y = -1

        if results.pose_landmarks:
            landmarks = results.pose_landmarks.landmark
            center_x, center_y, pixel_depth = self.extract_pose(landmarks, image, depth_image)
            
            self.cmd.x = center_x - image.shape[1] // 2  # Error respecto al centro
            self.cmd.y = pixel_depth if np.isfinite(pixel_depth) else -1
            #rospy.loginfo(f"Publishing: Error: {self.cmd.x}, Distance: {self.cmd.y}")
            
        if self.show_img:
            cv2.circle(image, (center_x, center_y), 5, (0, 0, 255), -1)

            center_of_image = (image.shape[1] // 2, center_y)
            cv2.line(image, center_of_image, (center_x, center_y), (255, 0, 0), 2)

        
            cv2.imshow('Person Detection', image)
            cv2.waitKey(1)

        self.cmd_pub.publish(self.cmd)

    def extract_pose(self, landmarks, image, depth_image):
        # Extraer caderas
        # Obtener coordenadas normalizadas de las caderas
        left_hip = landmarks[self.mp_pose.PoseLandmark.LEFT_HIP]
        right_hip = landmarks[self.mp_pose.PoseLandmark.RIGHT_HIP]
        #center = (left_hip + right_hip) / 2
        center = left_hip
        image_height, image_width, _ = image.shape
        pixel_x, pixel_y = int(center.x * image_width), int(center.y * image_height)
        pixel_depth = depth_image[pixel_y, pixel_x] if 0 <= pixel_x < image_width and 0 <= pixel_y < image_height else -1
        return pixel_x, pixel_y, pixel_depth

rospy.init_node('Person_Detector')
cd = MoveNetDetector()
rospy.spin()
