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

# Configuración del sintetizador de voz
engine = pyttsx3.init()
engine.setProperty('rate', 150)  # Velocidad del habla
engine.setProperty('volume', 1.0)  # Volumen

# Inicializa ROS y configura el topic
rospy.init_node('robot_voice_interface', anonymous=True)
log_pub = rospy.Publisher('/robot_logs', String, queue_size=10)
command_pub = rospy.Publisher('/robot_command', String, queue_size=10)

# Función para publicar, imprimir y sintetizar mensajes
def log_and_speak(message):
    # Publica en el topic de ROS
    rospy.loginfo(message)
    log_pub.publish(message)
    
    # Imprime en consola
    print(message)
    
    # Sintetiza el mensaje
    engine.say(message)
    engine.runAndWait()

# Ruta al modelo Vosk
model_path = "vosk-model-small-es-0.42"
model = Model(model_path)

# Cola para manejar el audio
q = queue.Queue()

# Configuración del micrófono
samplerate = 44100  # Frecuencia del micrófono (44100 Hz)
sd.default.samplerate = samplerate
sd.default.channels = 1  # Canal mono

# Inicializar el reconocedor de voz con la frecuencia correcta (16 kHz)
recognizer = KaldiRecognizer(model, 16000)  # Vosk siempre necesita 16 kHz

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
            log_and_speak(f"Texto reconocido: {texto}")
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
                    while True:
                        comando_especifico = reconocer_comando()

                        if "sígueme" in comando_especifico:
                            log_and_speak("De acuerdo, te sigo.")
                            command_pub.publish("start_follow")
                        elif "quédate aquí" in comando_especifico:
                            log_and_speak("De acuerdo, me quedo quieto.")
                            command_pub.publish("stop_follow")
                        elif "ve a la cocina" in comando_especifico:
                            log_and_speak("De acuerdo, me dirijo a la cocina.")
                            command_pub.publish("go_to")
                        elif "vuelve a la estación" in comando_especifico:
                            log_and_speak("De acuerdo, me dirijo a la estación de carga.")
                            command_pub.publish("back_station")
                        elif "adiós" in comando_especifico:
                            log_and_speak("Adiós")
                            command_pub.publish("goodbye")
                            return
                        else:
                            log_and_speak("Comando no reconocido.")
                elif "adiós" in comando_activacion:
                    log_and_speak("Saliendo del programa.")
                    break
    except Exception as e:
        log_and_speak(f"Error durante la ejecución: {e}")

if __name__ == "__main__":
    main()
