#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Image
import cv_bridge
from geometry_msgs.msg import Point
from std_msgs.msg import String
import cv2
import numpy as np
import tensorflow as tf
from message_filters import ApproximateTimeSynchronizer, Subscriber

from main import TOPIC_PERSONPOSE, TOPIC_COMMAND, START_DETECTION_CMD, STOP_DETECTION_CMD, TOPIC_RGBCAM, TOPIC_DEPTHCAM

EDGES = {
    (0, 1): 'm', (0, 2): 'c', (1, 3): 'm', (2, 4): 'c', (0, 5): 'm', (0, 6): 'c',
    (5, 7): 'm', (7, 9): 'm', (6, 8): 'c', (8, 10): 'c', (5, 6): 'y', (5, 11): 'm',
    (6, 12): 'c', (11, 12): 'y', (11, 13): 'm', (13, 15): 'm', (12, 14): 'c', (14, 16): 'c'
}

#  rostopic pub /robot_cmd std_msgs/String "start_detection"



class MoveNetDetector:
    def __init__(self):
        # Cargar modelo MoveNet
        self.interpreter = tf.lite.Interpreter(model_path='../tf_model/lite-model_movenet_singlepose_lightning_3.tflite')
        self.interpreter.allocate_tensors()
        self.input_details = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()

        self.bridge = cv_bridge.CvBridge()
        self.is_active = False
        self.cmd = None
        self.image_sub = None   
        self.depth_sub = None   
        self.cmd_pub = rospy.Publisher(TOPIC_PERSONPOSE, Point, queue_size=10)

        rospy.Subscriber(TOPIC_COMMAND, String, self.handler_callback)
        rospy.loginfo("Person detection waiting...")

    def handler_callback(self, msg):
        """
        Maneja cuándo este nodo debe suscribirse a los temas de imagen o no.
        """
        if msg.data == START_DETECTION_CMD and not self.is_active:
            self.is_active = True
            self.image_sub = Subscriber(TOPIC_RGBCAM, Image)
            self.depth_sub = rospy.Subscriber(TOPIC_DEPTHCAM, Image, self.detect_person)
            #ats = ApproximateTimeSynchronizer([self.image_sub, self.depth_sub], queue_size=10, slop=0.05)
            #ats.registerCallback(self.callback)
            #rospy.loginfo("PERSON DETECTION: Visual control ON")

        elif msg.data == STOP_DETECTION_CMD and self.is_active:
            self.is_active = False
            self.image_sub.unregister()
            self.depth_sub.unregister()
            rospy.loginfo("PERSON DETECTION: Visual control OFF")

    def callback(self, rgb_msg, depth_msg):
        rgb_image = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
        depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        self.detect_person(rgb_image, depth_image)

    def detect_person(self, msg):
        # Preprocesar la imagen para MoveNet
        depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        input_image = self.preprocess_depth_image(depth_image)
        #input_image = cv2.resize(depth_image, (192, 192))
        input_image = tf.image.resize_with_pad(np.expand_dims(input_image, axis=0), 192, 192)
        input_image = tf.cast(input_image, dtype=tf.float32)

        # Hacer predicciones
        self.interpreter.set_tensor(self.input_details[0]['index'], np.array(input_image))
        self.interpreter.invoke()
        keypoints_with_scores = self.interpreter.get_tensor(self.output_details[0]['index'])

        # Procesar resultados
        keypoints = np.squeeze(keypoints_with_scores)
        center_x, center_y, pixel_depth = self.extract_pose(keypoints, depth_image, depth_image)

        self.cmd = Point()
        self.cmd.x = center_x - depth_image.shape[1] // 2  # Error respecto al centro
        self.cmd.y = pixel_depth if np.isfinite(pixel_depth) else -1
        rospy.loginfo(f"Publishing: Error: {self.cmd.x}, Distance: {self.cmd.y}")
        self.cmd_pub.publish(self.cmd)

        depth_image_copy = depth_image.copy()
        cv2.circle(depth_image_copy, (center_x, center_y), 5, (0, 0, 255), -1)

        center_of_image = (depth_image_copy.shape[1] // 2, center_y)
        cv2.line(depth_image_copy, center_of_image, (center_x, center_y), (255, 0, 0), 2)
        # Dibujar pose
        #self.draw_pose(image, keypoints, confidence_threshold=0.3)

        cv2.imshow('Person Detection', depth_image_copy)
        cv2.waitKey(1)

    def extract_pose(self, keypoints, image, depth_image):
        # Extraer caderas
        left_hip = keypoints[11][:2]
        right_hip = keypoints[12][:2]
        #center = (left_hip + right_hip) / 2
        center = (left_hip)
        image_height, image_width = image.shape
        pixel_x, pixel_y = int(center[1] * image_width), int(center[0] * image_height)
        pixel_depth = depth_image[pixel_y, pixel_x] if 0 <= pixel_x < image_width and 0 <= pixel_y < image_height else -1
        return pixel_x, pixel_y, pixel_depth


    def draw_pose(self, frame, keypoints, confidence_threshold=0.3):
        for edge, color in EDGES.items():
            p1, p2 = edge
            y1, x1, c1 = keypoints[p1]
            y2, x2, c2 = keypoints[p2]
            if c1 > confidence_threshold and c2 > confidence_threshold:
                cv2.line(frame, (int(x1), int(y1)), (int(x2), int(y2)), (255, 0, 0), 2)
        for kp in keypoints:
            ky, kx, kp_conf = kp
            if kp_conf > confidence_threshold:
                cv2.circle(frame, (int(kx), int(ky)), 4, (0, 255, 0), -1)


    def preprocess_depth_image(self, depth_image):
        """
        Convierte la imagen de profundidad en un formato compatible con MoveNet.
        - Normaliza la profundidad a un rango de 0-255.
        - Duplica los valores en 3 canales para simular RGB.
        """
        # Normalizar los valores de profundidad al rango 0-255
        depth_image_normalized = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX)

        # Convertir la imagen a un formato de 3 canales
        depth_image_rgb = cv2.merge([depth_image_normalized] * 3).astype(np.uint8)
        return depth_image_rgb
    
rospy.init_node('Person_Detector')
cd = MoveNetDetector()
rospy.spin()
