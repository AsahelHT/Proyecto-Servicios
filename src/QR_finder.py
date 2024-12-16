#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import random
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from pyzbar.pyzbar import decode
import actionlib
import cv2
import math
import os
import numpy as np
import yaml
from PIL import Image as PILImage
import rospkg



from main import WAYPOINT_PATH, TOPIC_VEL, TOPIC_RGBCAM, TOPIC_LOGS, TOPIC_PRIMG, TOPIC_COMMAND, TOPIC_AMCLPOS


class QRCodeApproachLogger:
    def __init__(self):
        # Inicializar nodo
        rospy.init_node('qr_code_approach_logger', anonymous=True)
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        # Espera un máximo de 10 segundos para que el servidor esté disponible
        if not self.client.wait_for_server(timeout=rospy.Duration(5)):
            rospy.logerr("'move_base' server not available")

        # Límites del mapa (ajústalos a tu mapa)
        self.map_limits = {
            "x_min": -6.0, "x_max": 6.0,
            "y_min": -6.0, "y_max": 6.0
        }


        

        #WIP
        rospack = rospkg.RosPack()
        package_path = rospack.get_path('proyecto_servicios')
        folder_path = package_path + "/nav_maps"
        yaml_file = "mapa_campo.yaml"

       
        #map_path = "../nav_maps/mapa_campo.yaml"
       
        self.map_limits = self.get_map_limits(folder_path,yaml_file)

        # Límites del mapa (ajústalos a tu mapa)
        

        # Variables
        self.bridge = CvBridge()
        self.current_pose = None
        self.log_file_path = WAYPOINT_PATH
        self.active = True
        self.target_qr = None
        self.qr_size_threshold = 0.2  # Proporción mínima del QR en relación al ancho de la imagen
        self.image_width = None
        self.image_height = None
        self.reached_target = False
        self.robot_speed = 0.5  # Velocidad lineal del robot
        self.goal_cancel = False
        # Publicadores y suscriptores
        self.cmd_vel_pub = rospy.Publisher(TOPIC_VEL, Twist, queue_size=10)
        
        rospy.Subscriber(TOPIC_RGBCAM, Image, self.image_callback)
        rospy.Subscriber(TOPIC_AMCLPOS, PoseWithCovarianceStamped, self.update_pose)
        rospy.Subscriber(TOPIC_COMMAND, String, self.command_callback)
        self.image_pub = rospy.Publisher(TOPIC_PRIMG, Image, queue_size=10) 

        # Crear archivo o conservar contenido existente
        if not os.path.exists(self.log_file_path):
            with open(self.log_file_path, "w") as f:
                f.write("QR Code, Position (x, y, z), Orientation (x, y, z, w)\n")

        rospy.loginfo("Nodo QR Code Approach Logger inicializado.")

    def get_map_limits(self,path,filename):
    # Leer archivo YAML
        map_yaml_path=path+"/"+filename
        with open(map_yaml_path, 'r') as file:
            map_data = yaml.safe_load(file)

        # Obtener parámetros del mapa
        image_file = path+"/"+map_data['image']
        print(image_file)
        resolution = map_data['resolution']
        origin = map_data['origin']  # [x_min, y_min, theta]

        # Leer la imagen del mapa para obtener dimensiones
        with PILImage.open(image_file) as img:
            width, height = img.size

        # Calcular límites del mapa
        x_min, y_min = origin[:2]
        x_max = x_min + width * resolution
        y_max = y_min + height * resolution

        return {
            "x_min": x_min,
            "x_max": x_max,
            "y_min": y_min,
            "y_max": y_max
        }

    def update_pose(self, msg):
        # Actualizar la posición y orientación del robot
        self.current_pose = msg.pose.pose

    def command_callback(self, msg):
        # Comandos para activar o desactivar el procesamiento
        if msg.data == "start":
            rospy.loginfo("Activando detección de QR.")
            self.active = True
        elif msg.data == "stop":
            rospy.loginfo("Desactivando detección de QR.")
            self.active = False
            self.set_current_position_as_goal()

    def update_pose(self, msg):
        # Actualizar la posición y orientación del robot
        self.current_pose = msg.pose.pose

    def move_to_random_point(self):
        """Genera y envía un objetivo aleatorio al stack de navegación."""
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        # Generar un punto aleatorio dentro de los límites
        x = random.uniform(self.map_limits["x_min"], self.map_limits["x_max"])
        y = random.uniform(self.map_limits["y_min"], self.map_limits["y_max"])

        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.w = 1.0  # Orientación sin rotación

        rospy.loginfo(f"Enviando objetivo aleatorio: ({x:.2f}, {y:.2f})")
        self.client.send_goal(goal)
        self.client.wait_for_result()

        if self.client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("Objetivo alcanzado.")
            self.goal_cancel = False
        else:
            rospy.logwarn("Fallo al alcanzar el objetivo.")
            self.goal_cancel = False

    def set_current_position_as_goal(self):
        """Establece la posición actual del robot como un nuevo objetivo."""
        rospy.loginfo("Obteniendo la posición actual del robot...")

        # Asegúrate de que la pose actual esté disponible
        if self.current_pose is None:
            rospy.logwarn("No se ha recibido la posición actual del robot.")
            return

        # Crear un nuevo objetivo basado en la posición actual
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"  # El marco debe ser consistente con tu sistema
        goal.target_pose.header.stamp = rospy.Time.now()

        # Usar la posición y orientación actual del robot
        goal.target_pose.pose.position.x = self.current_pose.position.x
        goal.target_pose.pose.position.y = self.current_pose.position.y
        goal.target_pose.pose.position.z = self.current_pose.position.z
        goal.target_pose.pose.orientation.x = self.current_pose.orientation.x
        goal.target_pose.pose.orientation.y = self.current_pose.orientation.y
        goal.target_pose.pose.orientation.z = self.current_pose.orientation.z
        goal.target_pose.pose.orientation.w = self.current_pose.orientation.w

        # Enviar el nuevo objetivo al cliente de navegación
        self.client.send_goal(goal)
        rospy.loginfo("Nuevo objetivo enviado: mantener la posición actual.")

        # Opcional: Esperar a que el cliente confirme que el objetivo ha sido alcanzado
        self.client.wait_for_result()
        rospy.loginfo("Robot detenido en la posición actual.")
        self.goal_cancel = False

    def image_callback(self, msg):
        if not self.active:
            return

        try:
            # Convertir la imagen de ROS a OpenCV
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.image_height, self.image_width = frame.shape[:2]

            # Decodificar códigos QR en el frame
            for barcode in decode(frame):
                # Obtener texto y tipo del QR
                code_text = barcode.data.decode('utf-8')
                points = barcode.polygon

                if len(points) != 4:
                    continue  # Saltar detecciones no válidas

                # Calcular el tamaño del QR en píxeles
                qr_width_px = math.sqrt((points[1].x - points[0].x)**2 + (points[1].y - points[0].y)**2)

                # Comprobar si el QR ocupa suficiente espacio en la imagen
                

                # Comprobar si el QR ya ha sido registrado
                if self.is_qr_logged(code_text):
                    rospy.loginfo(f"QR detectado: {code_text}. Ya guardado, ignorando..")
                    continue
                    

                # Si no hay un QR objetivo, establecerlo
                if self.target_qr is None:
                    self.target_qr = code_text
                    rospy.loginfo(f"Nuevo QR detectado: {self.target_qr}.")
                    self.goal_cancel = True
                    self.reached_target = False
                    self.set_current_position_as_goal()

                if self.goal_cancel:
                    rospy.loginfo(f"Esperando a cancelar goal de movimiento")
                else:
                    rospy.loginfo(f"Aproximandose a QR: {self.target_qr}.")
                    # Mantener el QR centrado
                    qr_center_x = sum([point.x for point in points]) / 4.0
                    image_center_x = self.image_width / 2.0
                    center_error = qr_center_x - image_center_x

                    if abs(center_error) > 50:  # Ajustar tolerancia
                        self.rotate_towards_qr(center_error)
                        
                    else:
                        # Si el QR está centrado, avanzar
                        if qr_width_px < self.image_width * self.qr_size_threshold:
                            
                            self.move_towards_qr()
                        else:
                            # Detenerse y registrar el QR
                            self.stop_robot()
                            if not self.is_qr_logged(code_text):
                                self.log_qr_data(self.target_qr, self.current_pose.position, self.current_pose.orientation)
                            self.target_qr = None  # Resetear objetivo
                            self.reached_target = True
            

                # Dibujar un recuadro y el texto del QR en la imagen
                pts = [(point.x, point.y) for point in points]
                cv2.polylines(frame, [np.array(pts, np.int32)], True, (0, 255, 0), 2)
                cv2.putText(frame, f"QR: {code_text}", (pts[0][0], pts[0][1] - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

            # Mostrar la imagen
            cv2.imshow("QR Code Scanner", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                rospy.loginfo("Cerrando programa por tecla 'q'.")
                rospy.signal_shutdown("Tecla 'q' presionada.")
        except Exception as e:
            rospy.logerr(f"Error procesando la imagen: {e}")

    def publish_processed_image(self, frame):
        """Convierte la imagen procesada a formato ROS y la publica."""
        try:
            processed_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.image_pub.publish(processed_msg)
        except CvBridgeError as e:
            rospy.logerr(f"Error al publicar la imagen procesada: {e}")

    def is_qr_logged(self, code_text):
        # Comprobar si el QR ya está registrado en el archivo
        if os.path.exists(self.log_file_path):
            with open(self.log_file_path, "r") as f:
                for line in f:
                    if code_text in line:
                        return True
        return False

    def log_qr_data(self, code_text, position, orientation):
        # Registrar el QR con la posición y orientación en el archivo
        with open(self.log_file_path, "a") as f:
            f.write(f"{code_text}, ({position.x:.2f}, {position.y:.2f}, {position.z:.2f}), "
                    f"({orientation.x:.2f}, {orientation.y:.2f}, {orientation.z:.2f}, {orientation.w:.2f})\n")
        rospy.loginfo(f"QR '{code_text}' registrado en posición ({position.x:.2f}, {position.y:.2f}, {position.z:.2f}) "
                      f"y orientación ({orientation.x:.2f}, {orientation.y:.2f}, {orientation.z:.2f}, {orientation.w:.2f}).")

    def rotate_towards_qr(self, error):
        # Rotar el robot para centrar el QR en la imagen
        twist = Twist()
        twist.angular.z = -0.005 * error  # Ganancia proporcional
        self.cmd_vel_pub.publish(twist)

    def move_towards_qr(self):
        # Mover el robot hacia adelante
        twist = Twist()
        twist.linear.x = self.robot_speed
        self.cmd_vel_pub.publish(twist)

    def stop_robot(self):
        # Detener el movimiento del robot
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)

    def start(self):
        """Inicia el movimiento aleatorio por el mapa."""
        rospy.loginfo("Iniciando movimiento aleatorio por el mapa buscando QRs...")
        while not rospy.is_shutdown():
            if self.target_qr is None:
                self.move_to_random_point()
            rospy.sleep(1)

if __name__ == "__main__":
    try:
        navigator = QRCodeApproachLogger()
        navigator.start()
    except rospy.ROSInterruptException:
        rospy.loginfo("Nodo terminado.")
