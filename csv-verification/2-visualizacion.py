import pandas as pd
import argparse
import os
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D

def load_data(csv_file):
    # Read the CSV file, skipping metadata rows
    data = pd.read_csv(csv_file, skiprows=6)

    # Check if the necessary columns exist
    if 'Time (Seconds)' not in data.columns or 'X' not in data.columns or 'Y' not in data.columns or 'Z' not in data.columns:
        print("The required columns are not present in the CSV file.")
        return None

    # Drop rows with NaN values in the important columns
    data = data.dropna(subset=['Time (Seconds)', 'X', 'Y', 'Z'])

    return data

def check_quaternions(data):
    # Check if quaternions are normalized
    if {'qW', 'qX', 'qY', 'qZ'}.issubset(data.columns):
        for index, row in data.iterrows():
            norm = (row['qX']**2 + row['qY']**2 + row['qZ']**2 + row['qW']**2)**0.5
            if abs(norm - 1) > 0.01:  # 1% tolerance
                print(f"Warning: The quaternion in frame {row['Frame']} is not normalized.")
    else:
        print("No quaternions found in the file.")

def plot_trajectory(data):

    # Extraer posiciones X, Y, Z
    x = data['X'].values
    y = data['Y'].values
    z = data['Z'].values

    # Aplicar estilo limpio
    plt.style.use("seaborn-v0_8-muted")

    # 2D Plot mejorado
    plt.figure(figsize=(8, 6))
    plt.gca().set_facecolor('white')  # Fondo blanco
    plt.plot(x, y, color='navy', linewidth=2, alpha=0.8)
    plt.xlabel('X [mm]', labelpad=5, fontsize=16)  # Agregar espacio extra para la etiqueta X
    plt.ylabel('Y [mm]', labelpad=5, fontsize=16)  # Agregar espacio extra para la etiqueta Z
    plt.grid(True, linestyle='--', alpha=0.6)
    # Aumentar tamaño de las marcas de los ejes
    plt.tick_params(axis='both', which='major', labelsize=14)

    # Guardar la figura 2D
    plt.savefig("2D.png", dpi=300, bbox_inches='tight', transparent=False)
    plt.clf()  # Limpiar la figura para la siguiente

    # 3D Plot mejorado
    fig = plt.figure(figsize=(8, 6))
    fig.patch.set_facecolor('white')  # Fondo de la figura blanco
    ax = fig.add_subplot(111, projection='3d')

    # Establecer las paredes 3D como blancas
    ax.w_xaxis.pane.fill = False  # Desactiva el pane de color
    ax.w_yaxis.pane.fill = False
    ax.w_zaxis.pane.fill = False

    ax.set_facecolor('white')  # Fondo blanco para el área de la gráfica

    # Crear degradado de color
    cmap = plt.get_cmap("viridis")
    colors = cmap(np.linspace(0, 1, len(x)))

    for i in range(len(x) - 1):
        #ax.plot(x[i:i+2], y[i:i+2], z[i:i+2], color=colors[i], linewidth=2, alpha=0.9)
        #ax.plot(x[i:i+2], z[i:i+2], y[i:i+2], color=colors[i], linewidth=2, alpha=0.9)  #tr
        ax.plot(z[i:i+2], x[i:i+2],y[i:i+2], color=colors[i], linewidth=2, alpha=0.9) #tt y tr

    # Ajustar etiquetas con espacio adicional para evitar corte
    ax.set_xlabel('Z [mm]', labelpad=8, fontsize=14)
    ax.set_ylabel('X [mm]', labelpad=8, fontsize=14)
    ax.set_zlabel('Y [mm]', labelpad=8, fontsize=14)
    # Aumentar tamaño de las marcas de los ejes
    plt.tick_params(axis='both', which='major', labelsize=12)

    # Ajustar el espaciado de los ticks en 3D (tr)
    #ax.set_yticks(np.arange(-100, 501, step=100))
    #ax.set_xticks(np.arange(-300, -700, step=-100))
    #ax.set_zticks(np.arange(950, 1351, step=100))
    
    # Ajustar el espaciado de los ticks en 3D (tt)
    ax.set_yticks(np.arange(50, 501, step=100))
    ax.set_xticks(np.arange(-651, -250, step=100))
    ax.set_zticks(np.arange(950, 1351, step=100))

    # Ajustar vista para mejor percepción de la trayectoria
    ax.view_init(elev=20, azim=125)

    # Guardar la figura 3D
    plt.savefig("3D.png", dpi=300, transparent=False)
    plt.clf()  # Limpiar la figura para evitar superposición



def main():
    parser = argparse.ArgumentParser(description="Analysis of OptiTrack position and rotation data")
    parser.add_argument('--input', required=True, help="Name of the dataset folder")
    args = parser.parse_args()

    # Path to the CSV file
    folder_path = f'/Volumes/SSD/archivos/KUKA_dev/environment_modeling/ROSBAGS/{args.input}/'
    csv_file = os.path.join(folder_path, 'groundtruth_raw.csv')

    # Load the data
    data = load_data(csv_file)
    if data is not None:
        check_quaternions(data)
        plot_trajectory(data)

if __name__ == "__main__":
    main()
