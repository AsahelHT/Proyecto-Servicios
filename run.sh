#!/bin/bash

# Comprobar que se ha pasado al menos un argumento
if [ $# -eq 0 ]; then
  echo "Use: $0  <option> <move_person> <rviz>"
  echo "Options:"
  echo "  heavy - Launches everything"
  echo "  light - Launches navigation, follow, voice control and GUI"
  echo "  minimal - Launches navigation, follow and console"
  echo "  qr - Launches navigation and QR finder"
  echo "Default: nodes:=light, move_person:=false, rviz:=false"
fi

RVIZ="${3:-}"
MODE="${1:-light}"
MOVEPERSON="${2:-}"

# Fuente de ROS (ajustar según tu configuración)
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
source ~/ROS_WS/devel/setup.bash

echo "Launching World..."
gnome-terminal --title="WORLD" -- bash  -c "roslaunch proyecto_servicios world.launch" &
WORLD_PID=$!  # Guardar el PID del terminal

# Añadir un retardo para permitir que el mundo se cargue
echo "Waiting for World to load..."
sleep 10


echo "Launching Navigation..."
# Verificar si el argumento es 'rviz'
if [ "$RVIZ" == "rviz" ]; then
  gnome-terminal --title="NAV" -- bash -c "roslaunch proyecto_servicios navigation.launch rviz:=true" &
else
  gnome-terminal --title="NAV" -- bash -c "roslaunch proyecto_servicios navigation.launch" &
fi

# Seleccionar el archivo .launch según el argumento
case "$MODE" in
  heavy)
    echo "Launching Nodes: Heavy..."
    gnome-terminal --title="NODES_HEAVY" -- bash  -c "roslaunch proyecto_servicios nodes_heavy.launch" &
    NODES_PID=$!
    ;;
  light)
    echo "Launching Nodes: Light..."
    gnome-terminal --title="NODES_LIGHT" -- bash  -c "roslaunch proyecto_servicios nodes_light.launch" &
    NODES_PID=$!
    ;;
  minimal)
    echo "Launching Nodes: Minimal..."
    gnome-terminal --title="NODES_MINIMAL" -- bash  -c "roslaunch proyecto_servicios nodes_minimal.launch" &
    NODES_PID=$!
    ;;
  qr)
    echo "Launching QR finder"
    gnome-terminal --title="QR_FINDER" -- bash -c "rosrun proyecto_servicios QR_finder.py" &
    NODES_PID=$!
    ;;
  *)
    echo "Invalid option: $1"
    echo "Valid options: heavy, light, minimal"
    exit 1
    ;;
esac

if [ "$MOVEPERSON" == "move_person" ]; then
  echo "Executing move_person node..."
  gnome-terminal --title="MOVE_PERSON" -- bash -c "rosrun proyecto_servicios move_person.py" &
fi
# Esperar a que el usuario presione una tecla
echo "Press any key to stop all nodes and close terminals..."
read -n 1 -s  # Espera silenciosa hasta que se presione una tecla

# Cerrar todos los terminales lanzados
echo "Closing terminals..."
rostopic pub /robot_cmd std_msgs/String "data: 'shutdown_state'" -1

# Opcional: Cerrar cualquier nodo ROS residual
rosnode kill /gazebo
rosnode kill /gazebo_gui
rosnode cleanup
