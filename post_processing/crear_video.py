import cv2
import numpy as np
import os
import argparse

def generate_video(associations_file, image_folder, output_video, fps=30):
    timestamps = []
    image_files = []
    
    # Leer el archivo de asociaciones
    with open(associations_file, 'r') as file:
        lines = file.readlines()
    
    for line in lines:
        if line.startswith("#") or not line.strip():
            continue
        parts = line.split()
        timestamps.append(float(parts[0]))  # Timestamp
        image_files.append(os.path.join(image_folder, parts[1].split('/')[1]))  # Ruta de la imagen
    
    if not image_files:
        print("No se encontraron imágenes en el archivo de asociaciones.")
        return
    
    # Leer la primera imagen para obtener el tamaño del video
    first_image = cv2.imread(image_files[0])
    if first_image is None:
        print(f"Error al leer la imagen: {image_files[0]}")
        return
    height, width, _ = first_image.shape
    
    # Definir el códec y el objeto VideoWriter
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter(output_video, fourcc, fps, (width, height))
    
    for img_file in image_files:
        frame = cv2.imread(img_file)
        if frame is None:
            print(f"Error al cargar la imagen {img_file}, se omitirá.")
            continue
        out.write(frame)
    
    out.release()
    print(f"Video guardado en {output_video}")

if __name__ == "__main__":
    BASE_PATH = "/home/samuel/dev/environment_modeling/ROSBAGS/"
    
    parser = argparse.ArgumentParser(description="Generar un video a partir de imágenes ordenadas por timestamp.")
    parser.add_argument("--input", required=True, help="Carpeta dentro de ROSBAGS donde se encuentran associations.txt y la carpeta rgb/")
    args = parser.parse_args()
    
    input_path = os.path.join(BASE_PATH, args.input)
    associations_file = os.path.join(input_path, "associations.txt")
    image_folder = os.path.join(input_path, "rgb")
    output_video = os.path.join(input_path, "output_video.mp4")
    
    generate_video(associations_file, image_folder, output_video)

#python3 script.py --input rosbag_data