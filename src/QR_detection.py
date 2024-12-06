import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
from pyzbar.pyzbar import decode

# Variable para almacenar el dato del primer código QR detectado
instruct_qr = None

# Crear una instancia de CvBridge
bridge = CvBridge()

# Variable de estado para controlar el procesamiento
active = False

# Crear un publicador para el tópico qr_info
qr_pub = rospy.Publisher('/qr_info', String, queue_size=10)

# Función para procesar comandos de activación/desactivación
def command_callback(msg):
    global active
    if msg.data == "start":
        rospy.loginfo("Activando detección de QR.")
        active = True
    elif msg.data == "stop":
        rospy.loginfo("Desactivando detección de QR.")
        active = False

# Función callback para procesar las imágenes
def image_callback(msg):
    global instruct_qr, active

    if not active:
        # Si el sistema no está activado, simplemente retorna
        return

    try:
        # Convertir la imagen de ROS a OpenCV
        frame = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Decodificar cualquier código QR presente en el fotograma
        for barcode in decode(frame):
            # Extraer el texto del código QR
            code_text = barcode.data.decode('utf-8')
            code_type = barcode.type

            if instruct_qr is None:
                instruct_qr = code_text
                rospy.loginfo(f"Primer código QR detectado: {instruct_qr}")

            # Publicar el contenido del QR al tópico qr_info
            qr_pub.publish(code_text)

            # Dibujar el recuadro alrededor del código QR y mostrar el texto
            points = barcode.polygon
            if len(points) == 4:  # Asegurarse de que el contorno tenga 4 puntos
                pts = [(point.x, point.y) for point in points]
                cv2.polylines(frame, [np.array(pts, np.int32)], True, (0, 255, 0), 2)
                # Agregar texto del código QR sobre el recuadro
                cv2.putText(frame, f"{code_type}: {code_text}", (pts[0][0], pts[0][1] - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

        # Mostrar el fotograma en una ventana
        cv2.imshow("QR Code Scanner", frame)

        # Salir del bucle si se presiona 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rospy.loginfo("Cerrando programa por tecla 'q'.")
            rospy.signal_shutdown("Tecla 'q' presionada.")
    except Exception as e:
        rospy.logerr(f"Error procesando la imagen: {e}")

# Inicializar el nodo de ROS
rospy.init_node('qr_code_scanner', anonymous=True)

# Suscribirse al tópico de comandos
rospy.Subscriber('/qr_scanner_command', String, command_callback)

# Suscribirse al tópico de la cámara
rospy.Subscriber('/camera/rgb/image_raw', Image, image_callback)

# Mantener el nodo corriendo
rospy.spin()

# Cerrar las ventanas al finalizar
cv2.destroyAllWindows()
