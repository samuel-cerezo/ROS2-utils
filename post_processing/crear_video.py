import cv2
import glob
import os
import argparse

def create_video(image_folder):
    """
    Crea un video a partir de imágenes en una carpeta dada.
    
    Args:
        image_folder (str): Ruta de la carpeta que contiene las imágenes.
    """
    # Obtener lista de imágenes ordenadas por timestamp
    image_files = sorted(glob.glob(os.path.join(image_folder, "*.png")))

    if not image_files:
        print(f"No se encontraron imágenes en {image_folder}")
        return

    # Leer la primera imagen para obtener dimensiones
    frame = cv2.imread(image_files[0])
    height, width, _ = frame.shape

    # Configurar el video writer
    output_file = os.path.join(image_folder, "output.mp4")
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # Para .mp4
    frame_rate = 30  # Ajusta según sea necesario
    video = cv2.VideoWriter(output_file, fourcc, frame_rate, (width, height))

    # Agregar imágenes al video
    for image in image_files:
        frame = cv2.imread(image)
        video.write(frame)

    video.release()
    cv2.destroyAllWindows()

    print(f"✅ Video guardado en: {output_file}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Generar video a partir de imágenes.")
    parser.add_argument("--input", type=str, required=True, help="Ruta de la carpeta con imágenes.")
    args = parser.parse_args()

    create_video(args.input)


#python3 script.py --input /ruta/a/las/imagenes