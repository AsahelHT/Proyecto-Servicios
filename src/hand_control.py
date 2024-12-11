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

from main import TOPIC_COMMAND, START_DETECTION_CMD, STOP_DETECTION_CMD, TOPIC_RGBCAM, MOVE_CLOSER_CMD, MOVE_AWAY_CMD, RESET_DIST_CMD

class GestureDetector:

    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.is_active = False
        self.simulation = False
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
            rospy.loginfo("GESTURE CONTROL: Visual control ON")
            
        elif msg.data == STOP_DETECTION_CMD and self.is_active:
            self.is_active = False
            self.image_sub.unregister()
            if self.cap:
                self.cap.release()
            cv2.destroyAllWindows()
            rospy.loginfo("GESTURE CONTROL: Visual control OFF")

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
                self.mp_drawing.draw_landmarks(frame, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)
                
                # Detecta el gesto usando los puntos clave de la mano
                # gesture = self.recognize_gesture(hand_landmarks, rgb_frame)
                gesture = self.detect_gestures(hand_landmarks)
                
                if gesture == MOVE_CLOSER_CMD:
                    print("Gesto detectado: v - Acercarse")
                elif gesture == MOVE_AWAY_CMD:
                    print("Gesto detectado: ^ - Alejarse")
                elif gesture == RESET_DIST_CMD:
                    print("Gesto detectado: .|. - Reset")

        
        # Muestra el frame con las anotaciones (opcional)
        #cv2.imshow('TurtleBot2 Gesture Detection', frame)
        #cv2.waitKey(1)

        return gesture


    def detect_gestures(self, hand_landmarks):
        gesture = "unknown"

        INDEX = [8, 7, 6, 5]  # Índice
        MIDDLE = [12, 11, 10, 9]  # Medio
        RING = [16, 15, 14, 13]  # Anular
        PINKY = [20, 19, 18, 17]  # Meñique
        THUMB = [4, 3, 2, 1]  # Pulgar

        # Gesto "V" victoria

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
        
        # Gesto "^" victoria
        
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
        
        # Comprobar si el dedo medio está extendido
        middle_extended = self.is_finger_extended_upwards(hand_landmarks.landmark, *MIDDLE)
        
        # Comprobar si los demás dedos están recogidos
        index_retracted = not self.is_finger_extended_upwards(hand_landmarks.landmark, *INDEX)
        ring_retracted = not self.is_finger_extended_upwards(hand_landmarks.landmark, *RING)
        pinky_retracted = not self.is_finger_extended_upwards(hand_landmarks.landmark, *PINKY)
        thumb_retracted = hand_landmarks.landmark[THUMB[0]].x > hand_landmarks.landmark[THUMB[3]].x

        if middle_extended and index_retracted and ring_retracted and pinky_retracted:
            gesture = RESET_DIST_CMD
            return gesture
   
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


"""
def recognize_gesture(self, hand_landmarks, frame):
        # Extrae las coordenadas de puntos clave del pulgar y el índice
        wrist = hand_landmarks.landmark[self.mp_hands.HandLandmark.WRIST]
        index_tip = hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_TIP]
        index_dip = hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_DIP]
        index_pip = hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_PIP]
        index_mcp = hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_MCP]

        # Convertir las coordenadas normalizadas a píxeles
        h, w, _ = frame.shape
        wrist_px = (int(wrist.x * w), int(wrist.y * h))
        index_tip_px = (int(index_tip.x * w), int(index_tip.y * h))
        index_pip_px = (int(index_pip.x * w), int(index_pip.y * h))
        dist_tip = self.calc_dist(wrist_px, index_tip_px)
        dist_pip = self.calc_dist(wrist_px, index_pip_px)
        # Detecta gestos básicos: "mano abierta" o "puño cerrado"
        
        #print("Tip: ", abs(index_tip.z), " dip: ", abs(index_dip.z), " pip: ", abs(index_pip.z), " mcp: ", abs(index_mcp.z), " wrist: ", abs(wrist.z))
        
        dist_media = (abs(index_tip.z) + abs(index_dip.z) + abs(index_pip.z) + abs(index_mcp.z) + abs(index_mcp.z) + abs(wrist.z)) / 6
        print("Dist media: ", dist_media)
        if dist_media > 0.1:
            if abs(index_tip.z) > abs(index_dip.z) and abs(index_dip.z) > abs(index_pip.z) and abs(index_pip.z) > abs(index_mcp.z) and abs(index_mcp.z) > abs(wrist.z):
                print("Pointing")
                self.pointing = True
                return "pointing"

        if self.pointing:
            if dist_tip > dist_pip:
                self.pointing == False
                return "mano_abierta"
            else:
                self.pointing == False
                return "puño_cerrado"
"""