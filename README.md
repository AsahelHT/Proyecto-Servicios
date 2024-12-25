<!-- Improved compatibility of back to top link: See: https://github.com/othneildrew/Best-README-Template/pull/73 -->
<a id="readme-top"></a>
<!--
*** Thanks for checking out the Best-README-Template. If you have a suggestion
*** that would make this better, please fork the repo and create a pull request
*** or simply open an issue with the tag "enhancement".
*** Don't forget to give the project a star!
*** Thanks again! Now go create something AMAZING! :D
-->



<!-- PROJECT SHIELDS -->
<!--
*** I'm using markdown "reference style" links for readability.
*** Reference links are enclosed in brackets [ ] instead of parentheses ( ).
*** See the bottom of this document for the declaration of the reference variables
*** for contributors-url, forks-url, etc. This is an optional, concise syntax you may use.
*** https://www.markdownguide.org/basic-syntax/#reference-style-links
-->


<!-- GETTING STARTED -->
## Introducción

Este repositorio contiene el paquete de ROS proyecto_servicios para la asignatura Robótica de Servicios.

### Prerequisitos

Estos son los siguientes requisitos que se necesitan para poder usar el paquete:
1. ROS Noetic para Ubuntu: [http://wiki.ros.org/noetic/Installation/Ubuntu](http://wiki.ros.org/noetic/Installation/Ubuntu)
2. Turtlebot2: 
Instalar el simulador de Turtlebot 2

Estos robots sí los tenemos en el laboratorio, de modo que lo que pruebes en el simulador luego podrás probarlo con los robots reales. Por desgracia, no tienen soporte oficial para Noetic, lo que nos va a obligar a compilar los fuentes.

Para descargar los fuentes de los diversos repositorios necesarios de manera automática necesitas bajarte primero este fichero tb2.rosinstall. Bájatelo y déjalo en tu directorio $HOME (tu directorio personal, o sea /home/tu_nombre_de_usuario).

* Instrucciones
  ```sh
  #Dependencias de paquetes de Ubuntu
  sudo apt install libusb-dev libftdi-dev python-is-python3 pyqt5-dev-tools

  #Dependencias de paquetes de Noetic
  sudo apt install ros-noetic-openslam-gmapping ros-noetic-joy ros-noetic-base-local-planner ros-noetic-move-base

  #Directorio para los fuentes
  mkdir $HOME/tb2_ws
  cd $HOME/tb2_ws

  #Se baja los fuentes de los repos especificados en el .rosinstall
  wstool init src ../tb2.rosinstall

  #Arregla un problema de dependencias
  rm -rf src/ar_track_alvar/ar_track_alvar

  #Compilar.
  catkin_make_isolated
  ```

3. Workspace de catkin previamente creado: [http://wiki.ros.org/catkin/Tutorials/create_a_workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)
4. Gazebo: [https://classic.gazebosim.org/tutorials?tut=install_ubuntu](https://classic.gazebosim.org/tutorials?tut=install_ubuntu) 
5. Mediapipe: [https://pypi.org/project/mediapipe/](https://pypi.org/project/mediapipe/)
6. Dependencias de python:
   ```sh
   pip install -r requirements.txt
   ```
   
### Instalación

Para instalar el paquete proyecto_servicios:

1. Clonar el repositorio dentro de la carpeta src del WS de catkin:
   ```sh
   git clone https://github.com/AsahelHT/proyecto_servicios.git
   ```
2. Cambiar el nombre del directorio descargado a proyecto_servicios (sólo si tiene otro nombre, como por ejemplo: proyecto_servicios-master)

3. Desde el directorio raiz del WS de catkin:
   ```sh
   # Compila el paquete de ROS
   catkin build proyecto_servicios

   # Actualiza el entorno
   source devel/setup.bash
   ```

Con estos pasos el paquete ya debería estar correctamente compilado y listo para su uso.
<p align="right">(<a href="#readme-top">back to top</a>)</p>


<!-- USAGE EXAMPLES -->
## Uso

Para usar este paquete se ha implementado un fichero run.sh que permite lanzar la aplicación de manera cómoda y sencilla:

1. Desde una terminal situada en la carpeta raiz del paquete: 
```sh
   ./run.sh
```
   Esto lanzará el paquete con los valores de configuración por defecto.

2. Este fichero admite argumentos de ejecución al lanzarlo desde terminal:
   ```sh
   ./run.sh <option> <move_person> <rviz>
   ```

      + Las opciones admitidas son las siguientes:
         1. minimal:
            ```sh
               ./run.sh minimal
            ```
            Se lanzan los nodos:
               - Deteccion de personas
               - Interfaz por línea de comandos
               - Nodo principal (main)

         2. light:
            ```sh
               ./run.sh light
            ```
            Se lanzan los nodos:
               - Deteccion de personas
               - Control de voz
               - Interfaz visual
               - Nodo principal (main)

         3. heavy:
            ```sh
               ./run.sh heavy
            ```
            Se lanzan todos los nodos:
               - Deteccion de personas
               - Control de voz
               - Control por gestos
               - Interfaz visual
               - Interfaz por línea de comandos
               - Nodo principal (main)

      + El argumento move_person lanzará el nodo de movimiento de persona en un entorno simulado de gazebo:
            ```sh
               ./run.sh <option> move_person
            ```
      + El argumento rviz lanzará el entorno de visualización rviz:
            ```sh
               ./run.sh <option> <move_person> rviz
            ```
<p align="right">(<a href="#readme-top">back to top</a>)</p>

## Workspace
proyecto_servicios
├── CMakeLists.txt
├── launch
│   ├── navigation.launch
│   ├── nodes_heavy.launch
│   ├── nodes_light.launch
│   ├── nodes_minimal.launch
│   ├── person_world.launch
│   └── world.launch
├── nav_maps
│   ├── mapa_aula.pgm
│   ├── mapa_aula.yaml
│   ├── mapa_campo.pgm
│   └── mapa_campo.yaml
├── obj_models
│   ├── person_walking
│   │   ├── ...
│   │   ...
│   ├── qr_cocina
│   │   ├── ...
│   │   ...
│   ├── qr_habitacion
│   │   ├── ...
│   │   ...
│   ├── qr_salon
│   │   ├── ...
│   │   ...
│   └── qr_wc
│       ├── ...
│       ...
├── package.xml
├── README.md
├── requirements.txt
├── run.sh
├── src
│   ├── bash_interface.py
│   ├── hand_control.py
│   ├── main.py
│   ├── move_person.py
│   ├── pDetector_mediapipeGPU.py
│   ├── pDetector_mediapipe.py
│   ├── pDetector_MoveNet.py
│   ├── qr_code_log.txt
│   ├── QR_detection.py
│   ├── QR_finder.py
│   ├── save_pos.py
│   ├── user_gui.py
│   └── voice_control.py
├── trained_models
│   ├── lite-model_movenet_singlepose_lightning_3.tflite
│   ├── pose_landmarker_full.task
│   ├── pose_landmarker_heavy.task
│   ├── pose_landmarker_lite.task
│   └── vosk-model-small-es-0.42
│       ├── ...
│       ...
└── worlds
    ├── campo.world
    ├── casa2_persona.world
    ├── casa2.world
    ├── casa3.world
    ├── casa_grande.world
    ├── empty_person.world
    └── person_world.world

<!-- ACKNOWLEDGMENTS -->
## Participantes

- Pablo Tarancón Meseguer
- Pelayo López González
- Christofher Javier Riofrio Cuadrado
- Asahel Hernández Torné

<p align="right">(<a href="#readme-top">back to top</a>)</p>

