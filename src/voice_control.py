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

from main import FOLLOW_ST, STOP_FOLLOW_CMD, SHUTDOWN_ST, MOVE_ST, BASE_ST, TOPIC_COMMAND, TOPIC_LOGS
# Configuración del sintetizador de voz
engine = pyttsx3.init()
engine.setProperty('rate', 120)  # Velocidad del habla
engine.setProperty('volume', 1.0)  # Volumen

# Inicializa ROS y configura el topic
rospy.init_node('robot_voice_interface', anonymous=True)
log_pub = rospy.Publisher(TOPIC_LOGS, String, queue_size=10)
command_pub = rospy.Publisher(TOPIC_COMMAND, String, queue_size=1)

# Función para publicar, imprimir y sintetizar mensajes
def log_and_speak(message):
    # Publica en el topic de ROS
    rospy.loginfo(message)
    log_pub.publish(message)
    
    # Imprime en consola
    #print(message)
    
    # Sintetiza el mensaje
    engine.say(message)
    engine.runAndWait()

# Ruta al modelo Vosk
model_name = "vosk-model-small-es-0.42"

model_path = rospy.get_param('~voice_model', "")

if model_path == "":
    model_path = model_name
else:
    model_path = model_path + '/' + model_name

print("Path:", model_path)
model = Model(model_path)

# Cola para manejar el audio
q = queue.Queue()

# Configuración del micrófono
samplerate = 44100  # Frecuencia del micrófono (44100 Hz)
sd.default.samplerate = samplerate
sd.default.channels = 1  # Canal mono

# Inicializar el reconocedor de voz con la frecuencia correcta (16 kHz)
recognizer = KaldiRecognizer(model, 16000)  # Vosk siempre necesita 16 kHz

# Patrón para capturar la acción y el lugar
patron = r"^(.*)\b(cocina|baño|sala|habitación|estación)\b$"
cafe = r"^(.*)\b(cafe)\b$"

# Callback para manejar el audio en tiempo real
def callback(indata, frames, time, status):
    if status:
        log_and_speak(f"Estado del micrófono: {status}")
    # Reajusta la frecuencia de 44100 Hz a 16000 Hz
    downsampled_data = resample(indata, int(len(indata) * 16000 / 44100))
    # Convertir a int16 (escala los valores de flotante a rango de int16)
    int16_data = (downsampled_data * 32767).astype(np.int16)
    q.put(int16_data)

# Función para reconocer comandos desde el micrófono
def reconocer_comando(timeout=30):
    start_time = time.time()
    while True:
        try:
            data = q.get(timeout=1)  # Evita bloqueos indefinidos
        except queue.Empty:
            continue

        if recognizer.AcceptWaveform(data.tobytes()):
            result = json.loads(recognizer.Result())
            texto = result.get("text", "").lower()
            return texto

        if time.time() - start_time > timeout:
            log_and_speak("Tiempo de espera agotado.")
            return ""

# Bucle principal para escuchar y procesar los comandos
def main():
    try:
        
        with sd.InputStream(callback=callback):
            log_and_speak("Escuchando...")

            while not rospy.is_shutdown():
                comando_activacion = reconocer_comando()
                
                if "hola robot" in comando_activacion:
                    log_and_speak("Hola Usuario, ¿qué quieres que haga?")
                    inicio = time.time() 
                    intervalo = random.uniform(25, 60)
                    while True:
                        tiempo_actual = time.time()
                        comando_especifico = reconocer_comando()

                        coincidencia = re.match(patron, comando_especifico)

                        if tiempo_actual - inicio >= intervalo:
                            inicio = tiempo_actual  # Reinicia el temporizador
                            log_and_speak("Disculpe tendria usted para un cafe")
                        
                        if coincidencia:
                            accion = coincidencia.group(1).strip()  # Todo antes del lugar
                            lugar = coincidencia.group(2)          # El lugar
                        else:
                            accion = comando_especifico
                            lugar = None

                        if "sígueme" in accion:
                            log_and_speak("De acuerdo, te sigo.")
                            command_pub.publish(FOLLOW_ST)
                        elif "quédate aquí" in accion:
                            log_and_speak("De acuerdo, me quedo quieto.")
                            command_pub.publish(STOP_FOLLOW_CMD)
                        elif lugar != None:
                            log_and_speak("De acuerdo, me dirijo a " + lugar)
                            command_pub.publish(MOVE_ST + ":" + lugar)
                        elif "vuelve a la estación" in accion:
                            log_and_speak("De acuerdo, me dirijo a la estación de carga.")
                            command_pub.publish(MOVE_ST + ":" + "estacion")
                        elif "que" in accion or "un que" in accion or "como" in accion:
                            log_and_speak("Un cafe")
                            intervalo = random.uniform(25, 60)
                        elif "adiós" in accion:
                            log_and_speak("Adiós")
                            command_pub.publish(SHUTDOWN_ST)
                            return
                        else:
                            rospy.loginfo("Comando no reconocido.")
                            log_pub.publish("Comando no reconocido.")
                elif "adiós" in comando_activacion:
                    log_and_speak("Saliendo del programa.")
                    break
    except Exception as e:
        log_and_speak(f"Error durante la ejecución: {e}")



if __name__ == "__main__":
    main()
