#!/usr/bin/env python3

import rospy
from std_msgs.msg import String  # Asegúrate de que el tipo de mensaje sea adecuado al contenido del topic
from geometry_msgs.msg import PoseStamped
import os

class RobotCommandLogger:
    def __init__(self):
        # Iniciar nodo
        rospy.init_node('robot_command_logger', anonymous=True)

        # Variables
        self.current_position = None
        self.log_file_path = "robot_commands_log.txt"

        # Crear archivo o limpiar si ya existe
        with open(self.log_file_path, "w") as f:
            f.write("Nombre del Sitio, Espacio, Coordenadas (x, y, z)\n")

        # Suscribirse al topic para la posición actual del robot (por ejemplo, el pose del robot)
        rospy.Subscriber("/amcl_pose", PoseStamped, self.update_position)

        # Suscribirse al topic de comandos
        rospy.Subscriber("/robot_command", String, self.log_command)

        rospy.loginfo("Nodo iniciado y suscrito a /robot_command y /amcl_pose")

    def update_position(self, msg):
        # Actualizar la posición del robot
        self.current_position = msg.pose.position

    def log_command(self, msg):
        # Leer comando del topic
        command = msg.data

        if self.current_position is None:
            rospy.logwarn("No se ha recibido la posición del robot aún.")
            return

        # Guardar en el archivo
        with open(self.log_file_path, "a") as f:
            f.write(f"{command}, Espacio, ({self.current_position.x:.2f}, {self.current_position.y:.2f}, {self.current_position.z:.2f})\n")

        rospy.loginfo(f"Comando registrado: {command}, en ({self.current_position.x:.2f}, {self.current_position.y:.2f}, {self.current_position.z:.2f})")

if __name__ == "__main__":
    try:
        logger = RobotCommandLogger()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Nodo terminado.")

