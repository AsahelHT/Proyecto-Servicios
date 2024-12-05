import rospy
from geometry_msgs.msg import Point, Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2

# Parámetros de control
CENTER_TOLERANCE_X = 100  # Tolerancia en píxeles para el eje X
TARGET_DISTANCE = 1.5  # Distancia deseada en metros
LINEAR_GAIN = 0.5  # Ganancia para el control de la velocidad lineal
ANGULAR_GAIN = 0.005  # Ganancia para el control de la velocidad angular
MAX_VSPEED = 1
MAX_WSPEED = 0.3

cmd_vel_pub = None
last_error = 1

def person_pose_callback(msg):
    """Callback para recibir la posición del pixel desde /person_pose."""
    global cmd_vel_pub, last_error

    error_x = msg.x
    error_y = msg.y
    pixel_depth = msg.y


    if error_x == -1 and pixel_depth == -1:   # Person detection has not detected a person
        twist = Twist()
        twist.linear.x = 0

        if last_error > 0:
            twist.angular.z = -MAX_WSPEED 
        elif last_error < 0:
            twist.angular.z = MAX_WSPEED 

        print(last_error)
        rospy.loginfo(f"IDLE MOV: Velocidades: Linear X: {twist.linear.x:.2f}, Angular Z: {twist.angular.z:.2f}")
        cmd_vel_pub.publish(twist)
    # Verificar que el pixel esté dentro de los límites de la imagen
    else:
        # Obtener la profundidad en el pixel correspondiente
        
        twist = Twist()

        # Control angular para corregir el error
        if abs(error_x) > CENTER_TOLERANCE_X and error_x != -1:
            twist.angular.z = -ANGULAR_GAIN * error_x

            if twist.angular.z > MAX_WSPEED:
                twist.angular.z = MAX_WSPEED
                print("Limited")
            elif twist.angular.z < -MAX_WSPEED:
                twist.angular.z = -MAX_WSPEED
                print("-Limited")
        else:
            twist.angular.z = 0.0

        # Comprobar si la profundidad es válida
        if pixel_depth != -1:
            distance_error = pixel_depth - TARGET_DISTANCE  # Error respecto a la distancia deseada
            last_error = distance_error
            # Control lineal para mantener la distancia
            twist.linear.x = LINEAR_GAIN * distance_error
            
            twist.angular.z = twist.angular.z * distance_error
            if twist.linear.x > MAX_VSPEED:
                twist.linear.x = MAX_VSPEED
            elif twist.linear.x < -MAX_VSPEED:
                twist.linear.x = -MAX_VSPEED

            rospy.loginfo(f"Velocidades: Linear X: {twist.linear.x:.2f}, Angular Z: {twist.angular.z:.2f}")
        else: 
            twist.linear.x = 0
            rospy.loginfo(f"Velocidades: Linear X: {twist.linear.x:.2f}, Angular Z: {twist.angular.z:.2f}")

        cmd_vel_pub.publish(twist)
def main():
    global cmd_vel_pub
    
    rospy.init_node("distance_and_centering_controller")
    
    # Publicador para el tópico de velocidad
    cmd_vel_pub = rospy.Publisher("/cmd_vel_mux/input/navi", Twist, queue_size=10)

    # Suscriptor al tópico /person_pose
    rospy.Subscriber("/person_pose", Point, person_pose_callback)

    rospy.loginfo("Nodo de centrado y control de distancia inicializado. Esperando datos...")
    rospy.spin()

if __name__ == "__main__":
    main()
