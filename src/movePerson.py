#!/usr/bin/env python

import rospy
from pynput import keyboard
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState, ModelStates

# Variables globales para la posición del objeto
position = {'x': 0.0, 'y': 0.0, 'z': 0.0}
orientation = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}
model_name = "person_walking"  # Cambia esto al nombre de tu modelo en Gazebo
step_size = 0.1  # Tamaño del paso de movimiento


def get_initial_position():
    """
    Obtiene la posición inicial del objeto desde el mundo de Gazebo.
    """
    global position, orientation
    rospy.wait_for_message('/gazebo/model_states', ModelStates)
    try:
        msg = rospy.wait_for_message('/gazebo/model_states', ModelStates)
        if model_name in msg.name:
            index = msg.name.index(model_name)
            position['x'] = msg.pose[index].position.x
            position['y'] = msg.pose[index].position.y
            position['z'] = msg.pose[index].position.z
            orientation['x'] = msg.pose[index].orientation.x
            orientation['y'] = msg.pose[index].orientation.y
            orientation['z'] = msg.pose[index].orientation.z
            orientation['w'] = msg.pose[index].orientation.w
            rospy.loginfo(f"Posición inicial de '{model_name}': {position}")
        else:
            rospy.logerr(f"El modelo '{model_name}' no se encuentra en el mundo.")
            exit(1)
    except rospy.ROSException as e:
        rospy.logerr(f"Error al obtener la posición inicial: {e}")
        exit(1)


def move_object():
    """
    Enviar el nuevo estado al servicio de Gazebo para mover el objeto.
    """
    rospy.wait_for_service('/gazebo/set_model_state')
    try:
        set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

        # Configurar el estado del modelo
        state_msg = ModelState()
        state_msg.model_name = model_name
        state_msg.pose.position.x = position['x']
        state_msg.pose.position.y = position['y']
        state_msg.pose.position.z = position['z']
        state_msg.pose.orientation.x = orientation['x']
        state_msg.pose.orientation.y = orientation['y']
        state_msg.pose.orientation.z = orientation['z']
        state_msg.pose.orientation.w = orientation['w']
        state_msg.reference_frame = 'world'

        # Enviar el estado al simulador
        response = set_model_state(state_msg)
        if response.success:
            rospy.loginfo(f"El modelo se movió a: x={position['x']}, y={position['y']}, z={position['z']}")
        else:
            rospy.logerr(f"Error al mover el modelo: {response.status_message}")
    except rospy.ServiceException as e:
        rospy.logerr(f"Error al llamar al servicio: {e}")


def on_press(key):
    """
    Callback para manejar eventos de teclado cuando se presiona una tecla.
    """
    global position
    try:
        if key.char == 'w':  # Adelante
            position['x'] -= step_size
        elif key.char == 's':  # Atrás
            position['x'] += step_size
        elif key.char == 'a':  # Izquierda
            position['y'] -= step_size
        elif key.char == 'd':  # Derecha
            position['y'] += step_size
        move_object()
    except AttributeError:
        # Ignorar teclas especiales como Shift, Ctrl, etc.
        pass


def main():
    rospy.init_node('move_object_with_wasd')
    rospy.loginfo("Obteniendo posición inicial del objeto...")
    get_initial_position()

    rospy.loginfo("Controla el objeto usando las teclas WASD.")
    with keyboard.Listener(on_press=on_press) as listener:
        listener.join()


if __name__ == '__main__':
    main()
