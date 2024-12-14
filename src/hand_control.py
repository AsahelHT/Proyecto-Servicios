#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Image
import cv_bridge
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_msgs.msg import Bool
import cv2
import mediapipe as mp
import numpy as np
import math

from main import TOPIC_COMMAND, START_DETECTION_CMD, STOP_DETECTION_CMD, TOPIC_RGBCAM, MOVE_CLOSER_CMD, MOVE_AWAY_CMD, RESET_DIST_CMD, STOP_FOLLOW_CMD

class GestureDetector:

    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.is_active = False

        self.simulation = rospy.get_param('~sim', False)
        self.simulation = True
        self.cmd = None

        # Inicialización de MediaPipe para detección de manos
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands()
        self.mp_drawing = mp.solutions.drawing_utils

        self.image_sub = None      
        self.cmd_pub = rospy.Publisher(TOPIC_COMMAND, String, queue_size=10)

        rospy.Subscriber(TOPIC_COMMAND, String, self.cmd_callback)
        
        rospy.loginfo("Gesture control waiting...")

    def cmd_callback(self, msg):
        if msg.data == START_DETECTION_CMD and not self.is_active:
            self.is_active = True
            if self.simulation:
                self.cap = cv2.VideoCapture(0)
            self.image_sub = rospy.Subscriber(TOPIC_RGBCAM, Image, self.image_callback)
            rospy.loginfo("GESTURE CONTROL: Camera ON")
            
        elif msg.data == STOP_DETECTION_CMD and self.is_active:
            self.is_active = False
            self.image_sub.unregister()
            if self.cap:
                self.cap.release()
            cv2.destroyAllWindows()
            rospy.loginfo("GESTURE CONTROL: Camera OFF")

    def image_callback(self, msg):
        # Convierte el mensaje de imagen ROS a una imagen OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        if self.simulation: 
            # Asegúrate de que la cámara se haya inicializado correctamente
            if not self.cap.isOpened():
                rospy.logerr("Error: Unable to open camera")
                return
            ret, frame = self.cap.read()
            
            if not ret:
                rospy.logerr("Error: Unable to read camera")
                return
            
        frame = cv2.flip(frame, 1)

        self.cmd = self.gesture_cntrl(frame)

        if self.cmd != "unknown":
            self.cmd_pub.publish(self.cmd)

    # GESTURE CONTROL
    def gesture_cntrl(self, frame):
        # Convierte la imagen a RGB (MediaPipe requiere RGB)
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        result = self.hands.process(rgb_frame)
        gesture = "unknown"
        # Dibuja manos y detecta gestos específicos
        if result.multi_hand_landmarks:
            for hand_landmarks in result.multi_hand_landmarks:
                # Dibuja puntos clave y conexiones de la mano en el frame
                if self.simulation:
                    self.mp_drawing.draw_landmarks(frame, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)
                
                # Detecta el gesto usando los puntos clave de la mano
                # gesture = self.recognize_gesture(hand_landmarks, rgb_frame)
                gesture = self.detect_gestures(hand_landmarks)
                
                if gesture == MOVE_CLOSER_CMD:
                    print("Gesto detectado: v - Acercarse")
                elif gesture == MOVE_AWAY_CMD:
                    print("Gesto detectado: ^ - Alejarse")
                elif gesture == STOP_FOLLOW_CMD:
                    print("Gesto detectado: .|. - Dejar de seguir")
                elif gesture == RESET_DIST_CMD:
                    print("Gesto detectado: O - Reset distance")

        
        # Muestra el frame con las anotaciones (opcional)
        if self.simulation:
            cv2.imshow('TurtleBot2 Gesture Detection', frame)
            cv2.waitKey(1)

        return gesture


    def detect_gestures(self, hand_landmarks):
        gesture = "unknown"

        INDEX = [8, 7, 6, 5]  # Índice
        MIDDLE = [12, 11, 10, 9]  # Medio
        RING = [16, 15, 14, 13]  # Anular
        PINKY = [20, 19, 18, 17]  # Meñique
        THUMB = [4, 3, 2, 1]  # Pulgar

        """
        Gesto victoria "V" (ACERCARSE)
        """

        # Comprobar si el dedo medio está extendido
        middle_extended = self.is_finger_extended_upwards(hand_landmarks.landmark, *MIDDLE)
        index_extended = self.is_finger_extended_upwards(hand_landmarks.landmark, *INDEX)
        # Comprobar si los demás dedos están recogidos
        
        ring_retracted = not self.is_finger_extended_upwards(hand_landmarks.landmark, *RING)
        pinky_retracted = not self.is_finger_extended_upwards(hand_landmarks.landmark, *PINKY)
        thumb_retracted = hand_landmarks.landmark[THUMB[0]].x > hand_landmarks.landmark[THUMB[3]].x

        if middle_extended and index_extended and ring_retracted and pinky_retracted:
            gesture = MOVE_CLOSER_CMD
            return gesture
        
        """
        Gesto victoria invertido (ALEJARSE)
        """
        
        # Comprobar si el dedo medio está extendido
        middle_extended = self.is_finger_extended_downwards(hand_landmarks.landmark, *MIDDLE)
        index_extended = self.is_finger_extended_downwards(hand_landmarks.landmark, *INDEX)
        # Comprobar si los demás dedos están recogidos
        
        ring_retracted = not self.is_finger_extended_downwards(hand_landmarks.landmark, *RING)
        pinky_retracted = not self.is_finger_extended_downwards(hand_landmarks.landmark, *PINKY)
        thumb_retracted = hand_landmarks.landmark[THUMB[0]].x > hand_landmarks.landmark[THUMB[3]].x

        if middle_extended and index_extended and ring_retracted and pinky_retracted:
            gesture = MOVE_AWAY_CMD
            return gesture
        
        """
        Gesto dejar de seguir .|. (DEJAR DE SEGUIR)
        """

        # Comprobar si el dedo medio está extendido
        middle_extended = self.is_finger_extended_upwards(hand_landmarks.landmark, *MIDDLE)
        
        # Comprobar si los demás dedos están recogidos
        index_retracted = not self.is_finger_extended_upwards(hand_landmarks.landmark, *INDEX)
        ring_retracted = not self.is_finger_extended_upwards(hand_landmarks.landmark, *RING)
        pinky_retracted = not self.is_finger_extended_upwards(hand_landmarks.landmark, *PINKY)
        thumb_retracted = hand_landmarks.landmark[THUMB[0]].x > hand_landmarks.landmark[THUMB[3]].x

        if middle_extended and index_retracted and ring_retracted and pinky_retracted:
            gesture = STOP_FOLLOW_CMD
            return gesture
        
        """
        Gesto dejar de seguir O (RESET)
        """

        thumb_tip = hand_landmarks.landmark[THUMB[0]]
        index_tip = hand_landmarks.landmark[INDEX[0]]
        
        # Calcular la distancia entre la punta del pulgar y la punta del índice
        thumb_index_distance = self.calculate_distance(thumb_tip, index_tip)

        # Verificar que los otros dedos no interfieran con el gesto
        middle_extended = self.is_finger_extended_upwards(hand_landmarks.landmark, *MIDDLE)
        ring_extended = self.is_finger_extended_upwards(hand_landmarks.landmark, *RING)
        pinky_extended = self.is_finger_extended_upwards(hand_landmarks.landmark, *PINKY)

        # Si la distancia entre el pulgar y el índice es pequeña y los otros dedos están recogidos
        if thumb_index_distance < 0.05 and middle_extended and ring_extended and pinky_extended:
            gesture = RESET_DIST_CMD
            return gesture

    def calculate_distance(self, point1, point2):
        """
        Calcula la distancia euclidiana entre dos puntos.
        """
        return ((point1.x - point2.x) ** 2 + (point1.y - point2.y) ** 2 + (point1.z - point2.z) ** 2) ** 0.5
   
    def is_finger_extended_upwards(self, landmarks, tip, dip, pip, mcp):
        """
        Determina si un dedo específico está extendido basado en las posiciones de sus landmarks.
        """
        return landmarks[tip].y < landmarks[dip].y < landmarks[pip].y < landmarks[mcp].y
    
    def is_finger_extended_downwards(self, landmarks, tip, dip, pip, mcp):
        """
        Determina si un dedo específico está extendido basado en las posiciones de sus landmarks.
        """
        return landmarks[tip].y > landmarks[dip].y > landmarks[pip].y > landmarks[mcp].y
    
rospy.init_node('gesture_detector')
cd  = GestureDetector()
rospy.spin()       
