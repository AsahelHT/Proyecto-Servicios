#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String
from rich.console import Console
from rich.prompt import Prompt
from rich.panel import Panel
import re
import threading
import os

from main import rooms, TOPIC_COMMAND, TOPIC_LOGS, FOLLOW_ST, SHUTDOWN_ST, MOVE_ST
from main import STOP_FOLLOW_CMD, START_DETECTION_CMD, STOP_DETECTION_CMD, START_VOICE_CMD, STOP_VOICE_CMD

class CommandInterface:
    def __init__(self):
        self.console = Console()
        self.running = True
        self.camera_on = True
        self.voice_detection_on = True
        self.log_pub = rospy.Publisher(TOPIC_LOGS, String, queue_size=10)
        self.cmd_pub = rospy.Publisher(TOPIC_COMMAND, String, queue_size=1)
        rospy.Subscriber(TOPIC_COMMAND, String, self.cmd_callback)
        self.prompt_thread = None  # Hilo para el Prompt

    def cmd_callback(self, msg):
        if msg.data == SHUTDOWN_ST:
            self.console.print(":wave: [bold red]Apagando robot...[/bold red]")
            #self.cmd_pub.publish(SHUTDOWN_ST)
            self.running = False

    def handle_command(self, command):
        """
        Procesa los comandos ingresados por el usuario usando palabras clave.
        """
        command = command.lower()  # Convertir a minúsculas para evitar problemas de sensibilidad

        coincidencia = re.match(rooms, command)
        if coincidencia:
            accion = coincidencia.group(1).strip()  # Todo antes del lugar
            lugar = coincidencia.group(2)          # El lugar
        else:
            lugar = None

        if "sigue" in command or "sigueme" in command:
            self.console.print(":runner: [bold green]Siguiéndote...[/bold green]")
            self.cmd_pub.publish(FOLLOW_ST)
        elif "quieto" in command or "quedate" in command:
            self.console.print(":stop_sign: [bold yellow]Me quedo quieto.[/bold yellow]")
            self.cmd_pub.publish(STOP_FOLLOW_CMD)
        elif lugar:
            self.console.print(f":house: [bold blue]Yendo a {lugar}...[/bold blue]")
            self.cmd_pub.publish(MOVE_ST + ':' + lugar)
        elif "enciende" in command and "camara" in command:
            self.camera_on = True
            self.console.print(":camera: [bold green]Cámara encendida.[/bold green]")
            self.cmd_pub.publish(START_DETECTION_CMD)
        elif "apaga" in command and "camara" in command:
            self.camera_on = False
            self.console.print(":camera: [bold red]Cámara apagada.[/bold red]")
            self.cmd_pub.publish(STOP_DETECTION_CMD)
        elif "enciende" in command and "voz" in command:
            self.voice_detection_on = True
            self.cmd_pub.publish(START_VOICE_CMD)
            self.console.print(":microphone: [bold green]Detección de voz encendida.[/bold green]")
        elif "apaga" in command and "voz" in command:
            self.voice_detection_on = False
            self.cmd_pub.publish(STOP_VOICE_CMD)
            self.console.print(":microphone: [bold red]Detección de voz apagada.[/bold red]")
        elif "exit" in command:
            self.console.print(":wave: [bold red]Saliendo de la interfaz...[/bold red]")
            self.running = False
        elif "adios" in command or "apagar" in command:
            self.console.print(":wave: [bold red]Apagando robot...[/bold red]")
            self.cmd_pub.publish(SHUTDOWN_ST)
            self.running = False
        else:
            self.console.print(f":warning: [bold red]Comando desconocido: {command}[/bold red]")

    def prompt_loop(self):
        """
        Hilo para manejar el prompt interactivo.
        """
        while self.running:
            try:
                command = Prompt.ask("[bold magenta]Ingresa un comando[/bold magenta]")
                self.handle_command(command)
            except Exception as e:
                self.console.print(f":warning: [bold red]Error en el prompt: {e}[/bold red]")

    def start(self):
        """
        Inicia el bucle para recibir comandos.
        """
        self.console.print(Panel("[bold cyan]Interfaz de Comandos Iniciada[/bold cyan]\nEscribe 'exit' para salir.",
                                 title="[bold blue]Control de Robot[/bold blue]", expand=False))
        self.prompt_thread = threading.Thread(target=self.prompt_loop, daemon=True)
        self.prompt_thread.start()

        while self.running and not rospy.is_shutdown():
            rospy.sleep(0.1)  # Bucle principal para mantener ROS vivo

        rospy.signal_shutdown("Saliendo del programa")

if __name__ == "__main__":
    rospy.init_node('bash_interface')
    interface = CommandInterface()
    interface.start()
