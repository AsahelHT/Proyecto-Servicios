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

from main import TOPIC_PERSONPOSE, TOPIC_COMMAND, START_DETECTION_CMD, STOP_DETECTION_CMD, TOPIC_RGBCAM, TOPIC_DEPTHCAM, TOPIC_LOGS

EDGES = {
    (0, 1): 'm', (0, 2): 'c', (1, 3): 'm', (2, 4): 'c', (0, 5): 'm', (0, 6): 'c',
    (5, 7): 'm', (7, 9): 'm', (6, 8): 'c', (8, 10): 'c', (5, 6): 'y', (5, 11): 'm',
    (6, 12): 'c', (11, 12): 'y', (11, 13): 'm', (13, 15): 'm', (12, 14): 'c', (14, 16): 'c'
}

class MoveNetDetector:
    def __init__(self):
        # Cargar modelo MoveNet
        model_name = 'lite-model_movenet_singlepose_lightning_3.tflite'
        model_path = rospy.get_param('~tf_model', "")

        if model_path == "":
            model_path = model_name
        else:
            model_path = model_path + '/' + model_name

        self.interpreter = tf.lite.Interpreter(model_path=model_path)
                                                   
        self.interpreter.allocate_tensors()
        self.input_details = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()

        self.bridge = cv_bridge.CvBridge()
        self.is_active = False
        self.cmd = None
        self.image_sub = None   
        self.depth_sub = None   
        self.show_img = True
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
        # Preprocesar la imagen para MoveNet
        input_image = cv2.resize(image, (192, 192))
        input_image = tf.image.resize_with_pad(np.expand_dims(input_image, axis=0), 192, 192)
        input_image = tf.cast(input_image, dtype=tf.float32)

        # Hacer predicciones
        self.interpreter.set_tensor(self.input_details[0]['index'], np.array(input_image))
        self.interpreter.invoke()
        keypoints_with_scores = self.interpreter.get_tensor(self.output_details[0]['index'])

        # Procesar resultados
        keypoints = np.squeeze(keypoints_with_scores)
        center_x, center_y, pixel_depth = self.extract_pose(keypoints, image, depth_image,confidence_threshold=0.5)

        self.cmd = Point()
        print(center_x)
        
        self.cmd.x = center_x - image.shape[1] // 2  # Error respecto al centro
        self.cmd.y = pixel_depth if np.isfinite(pixel_depth) else -1
        #rospy.loginfo(f"Publishing: Error: {self.cmd.x}, Distance: {self.cmd.y}")
        self.cmd_pub.publish(self.cmd)
        print(self.cmd)
        # Dibujar pose
        #self.draw_pose(image, keypoints, confidence_threshold=0.3)

        if self.show_img:
            cv2.circle(image, (center_x, center_y), 5, (0, 0, 255), -1)

            center_of_image = (image.shape[1] // 2, center_y)
            cv2.line(image, center_of_image, (center_x, center_y), (255, 0, 0), 2)

            cv2.imshow('Person Detection', image)
            cv2.waitKey(1)

    def extract_pose(self, keypoints, image, depth_image, confidence_threshold=0.5):
        """
        Extrae la posición del centro de la persona basándose en las keypoints.
        Aplica un umbral de confianza para filtrar detecciones poco confiables.
        """
        # Extraer las keypoints de las caderas
        left_hip = keypoints[11][:2]
        left_hip_conf = keypoints[11][2]  # Puntaje de confianza para la cadera izquierda
        right_hip = keypoints[12][:2]
        right_hip_conf = keypoints[12][2]  # Puntaje de confianza para la cadera derecha

        # Verifica si las keypoints de las caderas son confiables
        if left_hip_conf < confidence_threshold or right_hip_conf < confidence_threshold:
            rospy.logwarn("Detección poco confiable. Ignorando frame.")
            return -1, -1, -1  # Retorna valores no válidos para esta detección

        # Calcular el centro entre las caderas
        center = (left_hip + right_hip) / 2
        image_height, image_width, _ = image.shape
        pixel_x, pixel_y = int(center[1] * image_width), int(center[0] * image_height)

        # Validar los límites de la imagen para evitar errores
        if not (0 <= pixel_x < image_width and 0 <= pixel_y < image_height):
            return -1, -1, -1

        # Extraer la profundidad en el centro
        pixel_depth = depth_image[pixel_y, pixel_x] if np.isfinite(depth_image[pixel_y, pixel_x]) else -1

        return pixel_x, pixel_y, pixel_depth

rospy.init_node('Person_Detector')
cd = MoveNetDetector()
rospy.spin()
