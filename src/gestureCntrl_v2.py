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

# /robot_command_follow
KP_SPEED = 0.4
MIN_SPEED = 0.1
MAX_SPEED = 1
SIDE_GAIN = 0.4         # value in %

class GestureDetector:

    def __init__(self):

        self.bridge = cv_bridge.CvBridge()
        self.is_active = False
        self.simulation = True
        self.cmd = None

        # Inicialización de MediaPipe para detección de manos
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands()
        self.mp_drawing = mp.solutions.drawing_utils

        self.image_sub = None      
        self.cmd_pub = rospy.Publisher('/robot_command', String, queue_size=10)

        rospy.Subscriber('/robot_command', String, self.follow_callback)
        
        rospy.loginfo("Gesture control waiting...")

    def follow_callback(self, msg):
        if msg.data == "start_follow" and not self.is_active:
            self.is_active = True
            if self.simulation:
                self.cap = cv2.VideoCapture(0)
            self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)
            rospy.loginfo("Visual control ON")
        elif msg.data == "stop_follow" and self.is_active:
            self.is_active = False
            self.image_sub.unregister()
            if self.cap:
                self.cap.release()
            cv2.destroyAllWindows()
            rospy.loginfo("Visual control OFF")

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

        if self.cmd != "uknown":
            self.cmd_pub.publish(self.cmd)

    # GESTURE CONTROL
    def gesture_cntrl(self, frame):
        # Convierte la imagen a RGB (MediaPipe requiere RGB)
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        result = self.hands.process(rgb_frame)
        gesture = "uknown"
        # Dibuja manos y detecta gestos específicos
        if result.multi_hand_landmarks:
            for hand_landmarks in result.multi_hand_landmarks:
                # Dibuja puntos clave y conexiones de la mano en el frame
                self.mp_drawing.draw_landmarks(frame, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)
                
                # Detecta el gesto usando los puntos clave de la mano
                # gesture = self.recognize_gesture(hand_landmarks, rgb_frame)
                gesture = self.detect_gestures(hand_landmarks)
                
                if gesture == "closer":
                    print("Gesto detectado: v - Acercarse")
                elif gesture == "away":
                    print("Gesto detectado: ^ - Alejarse")

        
        # Muestra el frame con las anotaciones (opcional)
        #cv2.imshow('TurtleBot2 Gesture Detection', frame)
        #cv2.waitKey(1)

        return gesture


    def detect_gestures(self, hand_landmarks):
        gesture = "Unknown"

        thumb_tip = hand_landmarks.landmark[self.mp_hands.HandLandmark.THUMB_TIP]
        thumb_ip = hand_landmarks.landmark[self.mp_hands.HandLandmark.THUMB_IP]
        thumb_mcp = hand_landmarks.landmark[self.mp_hands.HandLandmark.THUMB_MCP]
        thumb_cmc = hand_landmarks.landmark[self.mp_hands.HandLandmark.THUMB_CMC]

        index_tip = hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_TIP]
        index_pip = hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_PIP]

        middle_tip = hand_landmarks.landmark[self.mp_hands.HandLandmark.MIDDLE_FINGER_TIP]
        middle_pip = hand_landmarks.landmark[self.mp_hands.HandLandmark.MIDDLE_FINGER_PIP]

        ring_tip = hand_landmarks.landmark[self.mp_hands.HandLandmark.RING_FINGER_TIP]
        ring_pip = hand_landmarks.landmark[self.mp_hands.HandLandmark.RING_FINGER_PIP]

        pinky_tip = hand_landmarks.landmark[self.mp_hands.HandLandmark.PINKY_TIP]
        pinky_pip = hand_landmarks.landmark[self.mp_hands.HandLandmark.PINKY_PIP]

        # Gesto "V" victoria
        if (index_tip.y < hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_PIP].y and
            middle_tip.y < hand_landmarks.landmark[self.mp_hands.HandLandmark.MIDDLE_FINGER_PIP].y and
            ring_tip.y > hand_landmarks.landmark[self.mp_hands.HandLandmark.RING_FINGER_PIP].y and
            pinky_tip.y > hand_landmarks.landmark[self.mp_hands.HandLandmark.PINKY_PIP].y):

            gesture = "closer"

         # Condiciones para gesto "V" invertido
        index_down = index_tip.y > index_pip.y  # Índice apuntando hacia abajo
        middle_down = middle_tip.y > middle_pip.y  # Medio apuntando hacia abajo
        ring_folded = ring_tip.y < ring_pip.y  # Anular doblado
        pinky_folded = pinky_tip.y < pinky_pip.y  # Meñique doblado

        if index_down and middle_down and ring_folded and pinky_folded:
            gesture = "away"

        print(gesture)
        return gesture
   
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