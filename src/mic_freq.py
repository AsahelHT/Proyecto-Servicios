import sounddevice as sd

devices = sd.query_devices()
print("Dispositivos disponibles:")
for idx, device in enumerate(devices):
    print(f"{idx}: {device['name']} - {device['max_input_channels']} canales")

mic_id = int(input("Selecciona el ID del micrófono: "))
mic_info = devices[mic_id]

print(f"\nInformación del micrófono seleccionado:")
print(f"Nombre: {mic_info['name']}")
print(f"Frecuencia de muestreo predeterminada: {mic_info['default_samplerate']} Hz")
