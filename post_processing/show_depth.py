import cv2
import numpy as np
import argparse
import os
import matplotlib.pyplot as plt

# Configurar argumentos de entrada
parser = argparse.ArgumentParser(description="Visualizar imagen de profundidad con colormap")
parser.add_argument("--input", type=str, required=True, help="Ruta relativa al dataset de la imagen de profundidad")
args = parser.parse_args()

# Obtener ruta completa de la imagen
input_path = os.path.join("/home/samuel/dev/environment_modeling/ROSBAGS", args.input)

# Cargar imagen de profundidad (16UC1)
depth_img = cv2.imread(input_path, cv2.IMREAD_UNCHANGED)

# Verificar si la imagen se cargó correctamente
if depth_img is None:
    print(f"Error: No se pudo cargar la imagen en {input_path}")
    exit(1)

# Normalizar a rango [0, 255]
depth_normalized = cv2.normalize(depth_img, None, 0, 255, cv2.NORM_MINMAX)

# Convertir a 8 bits para visualización
depth_8bit = np.uint8(depth_normalized)

# Aplicar un colormap para convertir a RGB
depth_colormap = cv2.applyColorMap(depth_8bit, cv2.COLORMAP_JET)

# Mostrar la imagen usando matplotlib
plt.imshow(depth_colormap)
plt.colorbar()
plt.title("Depth Image")
plt.show()

# asume que la carpeta con el dataset esta en: /home/samuel/dev/environment_modeling/ROSBAGS
#python3 show_depth.py --input prueba_setup1/depth/1738241617.516180515.png
