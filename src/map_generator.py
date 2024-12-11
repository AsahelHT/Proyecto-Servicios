from PIL import Image

# Tamaño del mapa en píxeles (ancho x alto)
width, height = 200, 200  # Ajusta según sea necesario

# Crear una imagen completamente blanca (255 = libre)
image = Image.new('L', (width, height), 255)  # 'L' para escala de grises

# Guardar como archivo .pgm
image.save('empty_world.pgm')
print("Mapa vacío generado como 'empty_world.pgm'")
