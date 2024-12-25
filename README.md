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

### Simulación
Para usar este paquete se ha implementado un fichero run.sh que permite lanzar la aplicación de manera cómoda y sencilla en simulación:

+ Desde una terminal situada en la carpeta raiz del paquete:
   ```sh
      ./run.sh <option> <move_person> <rviz>
   ```
   1. Opcion minimal:
      ```sh
         ./run.sh minimal <move_person> <rviz>
      ```
      - Se lanzan los nodos:
         - Deteccion de personas
         - Interfaz por línea de comandos
         - Nodo principal (main)

   2. Opcion light:
      ```sh
         ./run.sh light <move_person> <rviz>
      ```
      - Se lanzan los nodos:
         - Deteccion de personas
         - Control de voz
         - Interfaz visual
         - Nodo principal (main)

   3. Opcion heavy:
      ```sh
         ./run.sh heavy <move_person> <rviz>
      ```
      - Se lanzan todos los nodos:
         - Deteccion de personas
         - Control de voz
         - Control por gestos
         - Interfaz visual
         - Interfaz por línea de comandos
         - Nodo principal (main)

   4. Opcion qr:
      ```sh
         ./run.sh qr <move_person> <rviz>
      ```
      Se lanza un módulo independiente para recorrer un entorno detectando QRs y guardar las posiciones donde se ha detectado el QR para poder navegar posteriormente hacia allí.


   5. El argumento move_person lanzará el nodo de movimiento de persona en un entorno simulado de gazebo:
         ```sh
            ./run.sh <option> move_person <rivz>
         ```
   6. El argumento rviz lanzará el entorno de visualización rviz:
         ```sh
            ./run.sh <option> <move_person> rviz
         ```

+ Si se desea lanzar un nodo por separado se puede seguir el procedimiento habitual de ROS, mediante rosnode:
   Ejemplo, lanzar move_person sin usar run.sh. (Debe realizarse source <path_to_catkin_ws>/devel/setup.bash primero)
   ```sh
      rosrun proyecto_servicios move_person.py
   ```
   
+ Para cerrar la aplicación se puede presionar cualquier tecla en el terminal donde se ejecutó el fichero run.sh
   + Alternativamente, mediante la interfaz visual (botón shutdown), la interfaz de comandos (escribiendo apagar) o mediante comando por voz (diciendo "adiós" o "apagar"), se cerrará la aplicación.
   + Si algun proceso no se cierra correctamente, simplemente se debe cerrar la terminal o abortar el proceso en la misma mediante CTRL + C.

#### Modficiación del entorno de simulación
Para cambiar el entorno de simulación de Gazebo a uno diferente del establecido por defecto existen dos opciones:

   1. Para poder lanzarlo mediante el run.sh:
      Dentro del fichero world.launch, cambiar el path que hace referencia al fichero del mundo .world deseado:
       <arg name="world_file" default="$(find proyecto_servicios)/worlds/<.world deseado>"/>

   2. Lanzar world.launch de manera independiente y pasar el .world deseado como argumento:
      ```sh
         roslaunch proyecto_servicios world.launch world_file:=<path to .world>
      ```
Para lanzar el stack de navegación en un entorno diferente al establecido por defecto existen dos opciones:
   
   1. Para poder lanzarlo mediante el run.sh:
      Dentro de navigation.launch modificar la siguiente linea para que haga referencia al fichero .yaml del mapa que se quiere usar:
   <arg name="map_file" default="$(find proyecto_servicios)/nav_maps/mapa_campo.yaml" />

   2. Lanzar navigation.launch de manera independiente y pasar el mapa deseado como argumento:
      ```sh
         roslaunch proyecto_servicios navigation.launch map_file:=<path to .yaml>
      ```

### Robot real
Si se pretende usar el paquete conjuntamente con el Turtlebot2 real, basta con lanzar la colección de nodos deseados mediante sus respectivos .launch en la máquina conectada al robot:
1. minimal:
```sh
   roslaunch proyecto_servicios nodes_minimal.launch
```
2. light:
```sh
   roslaunch proyecto_servicios nodes_light.launch
```
3. heavy:
```sh
   roslaunch proyecto_servicios nodes_heavy.launch
```

Para lanzar el stack de navegación en un entorno diferente al establecido por defecto existen dos opciones:
   
   1. Para poder lanzarlo mediante el run.sh:
      Dentro de navigation.launch modificar la siguiente linea para que haga referencia al fichero .yaml del mapa que se quiere usar:
   <arg name="map_file" default="$(find proyecto_servicios)/nav_maps/mapa_campo.yaml" />

   2. Lanzar navigation.launch de manera independiente y pasar el mapa deseado como argumento:
      ```sh
      roslaunch proyecto_servicios navigation.launch map_file:=<path to .yaml>
      ```
<p align="right">(<a href="#readme-top">back to top</a>)</p>

## Workspace

El workspace del paquete está distribuido de la siguiente manera:

<pre>
<span style="color: blue; font-weight: bold;">proyecto_servicios</span>
├── CMakeLists.txt
├── 📂 <span style="color: blue; font-weight: bold;">launch</span>
│   ├── navigation.launch
│   ├── nodes_heavy.launch
│   ├── nodes_light.launch
│   ├── nodes_minimal.launch
│   ├── person_world.launch
│   └── world.launch
├── 📂 <span style="color: blue; font-weight: bold;">nav_maps</span>
│   ├── mapa_aula.pgm
│   ├── mapa_aula.yaml
│   ├── mapa_campo.pgm
│   └── mapa_campo.yaml
├── 📂 <span style="color: blue; font-weight: bold;">obj_models</span>
│   ├── <span style="color: blue; font-weight: bold;">person_walking</span>
│   │   ├── ...
│   │   ...
│   ├── <span style="color: blue; font-weight: bold;">qr_cocina</span>
│   │   ├── ...
│   │   ...
│   ├── <span style="color: blue; font-weight: bold;">qr_habitacion</span>
│   │   ├── ...
│   │   ...
│   ├── <span style="color: blue; font-weight: bold;">qr_salon</span>
│   │   ├── ...
│   │   ...
│   └── <span style="color: blue; font-weight: bold;">qr_wc</span>
│       ├── ...
│       ...
├── package.xml
├── README.md
├── requirements.txt
├── run.sh
├── 📂 <span style="color: blue; font-weight: bold;">src</span>
│   ├── bash_interface.py
│   ├── hand_control.py
│   ├── main.py
│   ├── move_person.py
│   ├── pDetector_mediapipeGPU.py
│   ├── pDetector_mediapipe.py
│   ├── pDetector_MoveNet.py
│   ├── qr_code_log.txt
│   ├── QR_detection.py
│   ├── QR_finder.py
│   ├── save_pos.py
│   ├── user_gui.py
│   └── voice_control.py
├── 📂 <span style="color: blue; font-weight: bold;">trained_models</span>
│   ├── lite-model_movenet_singlepose_lightning_3.tflite
│   ├── pose_landmarker_full.task
│   ├── pose_landmarker_heavy.task
│   ├── pose_landmarker_lite.task
│   └── vosk-model-small-es-0.42
│       ├── ...
│       ...
└── 📂 <span style="color: blue; font-weight: bold;">worlds</span>
    ├── campo.world
    ├── casa2_persona.world
    ├── casa2.world
    ├── casa3.world
    ├── casa_grande.world
    ├── empty_person.world
    └── person_world.world
</pre>


1. En la carpeta /launch se encuentran todos los ficheros .launch implementados. Estos .launch son lanzados de manera automática por el fichero run.sh pero pueden ser lanzados independientemente mediante el paquete de ROS, roslaunch:
```sh
   roslaunch proyecto_servicios <launch file>
```

2. La carpeta /nav_maps contiene los mapas usados por el stack de navegación. Aquí deben situarse los mapas construidos.

3. /obj_models contiene los modelos de Gazebo usados en el mampa por defecto.

4. La carpeta /src contiene los códigos fuente que lanzan los distintos nodos y funcionalidades.

5. En /trained_models se incluyen los modelos y dependencias necesarias para poder usar el reconocmiento de voz y diferentes métodos de detección de personas (no necesarios si se está usando Mediapipe por defecto)

6. En /worlds contiene los ficheros .world que utiliza Gazebo para la simulación. Por defecto se lanza en simulación casa3.world.

<!-- ACKNOWLEDGMENTS -->
## Participantes

- Pablo Tarancón Meseguer
- Pelayo López González
- Christofher Javier Riofrio Cuadrado
- Asahel Hernández Torné

<p align="right">(<a href="#readme-top">back to top</a>)</p>

