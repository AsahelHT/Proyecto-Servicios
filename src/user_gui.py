#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QTextEdit, QFrame, QPushButton, QLineEdit
)
from PyQt5.QtCore import Qt, QThread, pyqtSignal
from PyQt5.QtGui import QPixmap, QImage
import cv2 as cv
from cv_bridge import CvBridge

from main import TOPIC_COMMAND, TOPIC_LOGS, TOPIC_RGBCAM, STOP_FOLLOW_CMD, FOLLOW_ST, MOVE_ST, SHUTDOWN_ST

class RosSubscriberThread(QThread):
    log_message_received = pyqtSignal(str)
    camera_image_received = pyqtSignal(object)

    def __init__(self):
        super().__init__()
        rospy.init_node('turtlebot_ui_node', anonymous=True)
        self.subscriber_log = rospy.Subscriber(TOPIC_LOGS, String, self.callback_log)
        self.subscriber_camera = rospy.Subscriber(TOPIC_RGBCAM, Image, self.callback_camera)

        self.bridge = CvBridge()
        self.publisher_command = rospy.Publisher(TOPIC_COMMAND, String, queue_size=10)  # Publicador para comandos
        self.publisher_log = rospy.Publisher(TOPIC_LOGS, String, queue_size=10)  # Publicador para log

    def callback_log(self, msg):
        self.log_message_received.emit(msg.data)

    def callback_camera(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        cv_image = cv.cvtColor(cv_image, cv.COLOR_BGR2RGB)  # Cambié cv2 a cv
        self.camera_image_received.emit(cv_image)

    def run(self):
        rospy.spin()

    def publish_command(self, command):
        self.publisher_command.publish(command)

    def publish_log(self, message):
        self.publisher_log.publish(message)

class TurtleBotControlUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("TurtleBot Control Interface")
        self.setGeometry(100, 100, 1200, 800)

        # Crear layout principal
        main_layout = QHBoxLayout(self)

        # Configuración de la sección izquierda (2/3 de la pantalla)
        left_frame = QFrame()
        left_frame.setFrameShape(QFrame.StyledPanel)
        left_layout = QVBoxLayout(left_frame)

        self.camera_label = QLabel("Imagen de la cámara")
        self.camera_label.setAlignment(Qt.AlignCenter)
        self.camera_label.setStyleSheet("background-color: lightblue; font-size: 18px;")
        
        # Aquí se añadirá un QLabel donde se mostrará la imagen (vacía por ahora)
        self.image_label = QLabel(self)
        self.image_label.setAlignment(Qt.AlignCenter)
        left_layout.addWidget(self.image_label)

        # Agregar botones debajo de la imagen de la cámara
        self.buttons_layout = QVBoxLayout()
        
        # Crear los botones
        self.follow_button = QPushButton('Follow')
        self.stop_button = QPushButton('Stop')
        self.goodbye_button = QPushButton('Goodbye')
        self.home_button = QPushButton('Home')

        # Añadir los botones al layout
        self.buttons_layout.addWidget(self.follow_button)
        self.buttons_layout.addWidget(self.stop_button)
        self.buttons_layout.addWidget(self.goodbye_button)
        self.buttons_layout.addWidget(self.home_button)

        left_layout.addLayout(self.buttons_layout)
        main_layout.addWidget(left_frame, stretch=2)

        # Configuración de la sección derecha (1/3 de la pantalla)
        right_frame = QFrame()
        right_frame.setFrameShape(QFrame.StyledPanel)
        right_layout = QVBoxLayout(right_frame)

        # Log de instrucciones y comandos
        self.log_text_edit = QTextEdit()
        self.log_text_edit.setReadOnly(True)
        self.log_text_edit.setPlaceholderText("Log de instrucciones y comandos...")
        right_layout.addWidget(self.log_text_edit)

        # Añadir la casilla "Go to..." y el botón
        self.go_to_label = QLabel("Go to...")
        self.go_to_input = QLineEdit(self)
        self.go_button = QPushButton("Go", self)
        self.go_button.clicked.connect(self.send_go_to_command)  # Llama a la función para enviar el comando

        right_layout.addWidget(self.go_to_label)
        right_layout.addWidget(self.go_to_input)
        right_layout.addWidget(self.go_button)

        main_layout.addWidget(right_frame, stretch=1)

        self.setLayout(main_layout)

        # Iniciar el hilo de ROS
        self.ros_thread = RosSubscriberThread()
        self.ros_thread.log_message_received.connect(self.update_log)
        self.ros_thread.camera_image_received.connect(self.update_camera_image)
        self.ros_thread.start()

        # Conectar botones con sus funciones de publicación
        self.follow_button.clicked.connect(self.send_follow_command)
        self.stop_button.clicked.connect(self.send_stop_command)
        self.goodbye_button.clicked.connect(self.send_goodbye_command)
        self.home_button.clicked.connect(self.send_home_command)

    def update_log(self, message):
        current_text = self.log_text_edit.toPlainText()
        new_text = current_text + "\n" + message
        self.log_text_edit.setPlainText(new_text)

    def update_camera_image(self, cv_image):
        height, width, channel = cv_image.shape
        bytes_per_line = 3 * width
        q_image = QImage(cv_image.data, width, height, bytes_per_line, QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(q_image)
        self.image_label.setPixmap(pixmap.scaled(self.image_label.size(), Qt.KeepAspectRatio))

    # Funciones que se llaman al pulsar los botones
    def send_follow_command(self):
        self.ros_thread.publish_command(FOLLOW_ST)  # Publica 'start_follow'
        #self.ros_thread.publish_log('Iniciando seguimiento...')

    def send_stop_command(self):
        self.ros_thread.publish_command(STOP_FOLLOW_CMD)  # Publica 'stop_follow'
        #self.ros_thread.publish_log('Deteniendo seguimiento...')

    def send_goodbye_command(self):
        self.ros_thread.publish_command(SHUTDOWN_ST)
        #self.ros_thread.publish_log('Goodbye...')

    def send_home_command(self):
        self.ros_thread.publish_command(MOVE_ST+':'+"estacion")
        #self.ros_thread.publish_log('Volviendo a home...')

    def send_go_to_command(self):
        # Obtener el texto de la casilla de entrada
        destination = self.go_to_input.text()
        
        # Crear el mensaje en el formato "go_to_<input>"
        if destination:
            go_to_message = MOVE_ST+":"+destination
            self.ros_thread.publish_command(go_to_message)  # Publica el comando 'go_to_<destination>'
            self.ros_thread.publish_log(f"[INFO]: Sending cmd: {go_to_message}")  # Log del mensaje enviado
            self.go_to_input.clear()  # Limpiar el campo de entrada después de enviar el comando


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = TurtleBotControlUI()
    window.show()
    sys.exit(app.exec_())
