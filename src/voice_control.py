#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String
import sounddevice as sd
from vosk import Model, KaldiRecognizer
import numpy as np
import queue
import json
import pyttsx3
import time
from scipy.signal import resample
import re
import random

from main import TOPIC_COMMAND, TOPIC_LOGS, rooms
from main import FOLLOW_ST, STOP_FOLLOW_CMD, SHUTDOWN_ST, MOVE_ST
from main import STOP_DETECTION_CMD, START_DETECTION_CMD, STOP_VOICE_CMD, START_VOICE_CMD


class VoiceControl:
    def __init__(self):
        self.engine = pyttsx3.init()
        self.engine.setProperty('rate', 120)  # Velocidad del habla
        self.engine.setProperty('volume', 1.0)  # Volumen

        self.log_pub = rospy.Publisher(TOPIC_LOGS, String, queue_size=10)
        self.command_pub = rospy.Publisher(TOPIC_COMMAND, String, queue_size=1)

        self.is_active = True
        self.cafe_on = True

        self.model_name = "vosk-model-small-es-0.42"

        self.model_path = rospy.get_param('~voice_model', "../trained_models")

        if self.model_path == "":
            self.model_path = self.model_name
        else:
            self.model_path = self.model_path + '/' + self.model_name

        self.model = Model(self.model_path)
        self.q = queue.Queue()

        # Configuración del micrófono
        self.samplerate = 44100  # Frecuencia del micrófono (44100 Hz)
        sd.default.samplerate = self.samplerate
        sd.default.channels = 1  # Canal mono
        self.recognizer = KaldiRecognizer(self.model, 16000)  # Vosk siempre necesita 16 kHz

        rospy.Subscriber(TOPIC_COMMAND, String, self.cmd_callback)
    # Función para publicar, imprimir y sintetizar mensajes
    def log_and_speak(self, message):
        # Publica en el topic de ROS
        rospy.loginfo(message)
        self.log_pub.publish(message)
        
        # Imprime en consola
        #print(message)
        
        # Sintetiza el mensaje
        self.engine.say(message)
        self.engine.runAndWait()

        # Callback para manejar el audio en tiempo real
    def audio_callback(self, indata, frames, time, status):
        if status:
            self.log_and_speak(f"Estado del micrófono: {status}")
        # Reajusta la frecuencia de 44100 Hz a 16000 Hz
        downsampled_data = resample(indata, int(len(indata) * 16000 / 44100))
        # Convertir a int16 (escala los valores de flotante a rango de int16)
        int16_data = (downsampled_data * 32767).astype(np.int16)
        self.q.put(int16_data)

    def cmd_callback(self, msg):
        if msg.data == START_VOICE_CMD and not self.is_active:
            self.is_active = True
            rospy.loginfo("VOICE CONTROL: ON")
            
        elif msg.data == STOP_VOICE_CMD and self.is_active:
            self.is_active = False
            rospy.loginfo("VOICE CONTROL: OFF")

        elif msg.data == SHUTDOWN_ST:
            self.engine.say("Apagando")
            self.engine.runAndWait()
            rospy.signal_shutdown("Apagando")

    def reconocer_comando(self, timeout=30):
        start_time = time.time()
        while True:
            try:
                data = self.q.get(timeout=1)  # Evita bloqueos indefinidos
            except queue.Empty:
                continue

            if self.recognizer.AcceptWaveform(data.tobytes()):
                result = json.loads(self.recognizer.Result())
                texto = result.get("text", "").lower()
                return texto

            if time.time() - start_time > timeout:
                self.log_and_speak("Tiempo de espera agotado.")
                return ""
            
    def main_loop(self):
        try:
            
            with sd.InputStream(callback=self.audio_callback):
                self.log_and_speak("Escuchando...")

                while not rospy.is_shutdown():
                    comando_activacion = self.reconocer_comando()
                    print(comando_activacion)
                    if "hola robot" in comando_activacion or "hola" in comando_activacion or "escuchame" in comando_activacion or "escucha" in comando_activacion or "oye" in comando_activacion or "oyeme" in comando_activacion:
                        self.is_active = True
                        if "hola robot" in comando_activacion or "hola" in comando_activacion:
                            self.log_and_speak("Hola usuario, ¿qué quieres que haga?")
                        elif "escucha" in comando_activacion or "escuchame" in comando_activacion or "oye" in comando_activacion or "oyeme" in comando_activacion:
                            self.log_and_speak("Te estoy escuchando")

                        if self.cafe_on:
                            inicio = time.time() 
                            intervalo = random.uniform(25, 60)

                        while self.is_active:
                           
                            comando_especifico = self.reconocer_comando()

                            if self.cafe_on:
                                tiempo_actual = time.time()
                                if tiempo_actual - inicio >= intervalo:
                                    inicio = tiempo_actual  # Reinicia el temporizador
                                    self.log_and_speak("Disculpe tendria usted para un cafe")

                            coincidencia = re.match(rooms, comando_especifico)
                        
                            if coincidencia:
                                accion = coincidencia.group(1).strip()  # Todo antes del lugar
                                lugar = coincidencia.group(2)          # El lugar
                            else:
                                accion = comando_especifico
                                lugar = None

                            if "sígueme" in accion:
                                self.log_and_speak("De acuerdo, te sigo.")
                                self.command_pub.publish(FOLLOW_ST)
                            elif "quédate aquí" in accion:
                                self.log_and_speak("De acuerdo, me quedo quieto.")
                                self.command_pub.publish(STOP_FOLLOW_CMD)
                            elif lugar != None:
                                self.log_and_speak("De acuerdo, me dirijo a " + lugar)
                                self.command_pub.publish(MOVE_ST + ":" + lugar)
                            elif "vuelve a la estación" in accion:
                                self.log_and_speak("De acuerdo, me dirijo a la estación de carga.")
                                self.command_pub.publish(MOVE_ST + ":" + "estacion")
                            elif "qué" in accion or "un qué" in accion or "cómo" in accion or "que" in accion or "un que" in accion or "como" in accion:
                                self.log_and_speak("Un cafe")
                                self.cafe_on = False
                            elif "no mires" in accion or ("apaga" in accion and "camara" in accion):
                                self.log_and_speak("De acuerdo, no miro")
                                self.command_pub.publish(STOP_DETECTION_CMD)
                            elif "mírame" in accion or ("apaga" in accion and "camara" in accion):
                                self.log_and_speak("De acuerdo, enciendo camara")
                                self.command_pub.publish(START_DETECTION_CMD)
                            elif "no" in accion and ("oigas" in accion or "escuches" in accion):
                                self.log_and_speak("De acuerdo, no te oigo")
                                self.is_active = False
                                break
                            elif "adiós" in accion:
                                self.log_and_speak("Adiós")
                                self.command_pub.publish(SHUTDOWN_ST)
                                rospy.signal_shutdown("Apagando")
                                break
                            else:
                                rospy.loginfo("Comando no reconocido.")
                                self.log_pub.publish("Comando no reconocido.")
                    if "adiós" in comando_activacion:
                        self.log_and_speak("Adiós.")
                        self.command_pub.publish(SHUTDOWN_ST)
                        rospy.signal_shutdown("Apagando")
                        break
        except Exception as e:
            self.log_and_speak(f"Error durante la ejecución: {e}")


rospy.init_node('voice_command_processor')
processor = VoiceControl()
processor.main_loop()

rospy.spin()    