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
    x = data['X'].values / 1000
    y = data['Y'].values / 1000
    z = data['Z'].values / 1000

    # Aplicar estilo limpio
    plt.style.use("seaborn-v0_8-muted")

    # 2D Plot mejorado
    plt.figure(figsize=(8, 6))
    plt.gca().set_facecolor('white')  # Fondo blanco
    plt.plot(x, y, color='navy', linewidth=3, alpha=0.8)
    plt.xlabel('X [m]', labelpad=5, fontsize=16, fontname='Times New Roman')  # Agregar espacio extra para la etiqueta X
    plt.ylabel('Y [m]', labelpad=5, fontsize=16, fontname='Times New Roman')  # Agregar espacio extra para la etiqueta Z
    # Formatear los ticks para que muestren solo un decimal
    plt.gca().set_xticklabels([f'{tick:.1f}' for tick in plt.gca().get_xticks()], fontname="Times New Roman", fontsize=14)
    plt.gca().set_yticklabels([f'{tick:.1f}' for tick in plt.gca().get_yticks()], fontname="Times New Roman", fontsize=14)


    plt.grid(True, linestyle='--', alpha=0.6)
    # Aumentar tamaño de las marcas de los ejes
    plt.tick_params(axis='both', which='major', labelsize=14)

    # Guardar la figura 2D
    #plt.savefig("2D.png", dpi=300, bbox_inches='tight', transparent=False)
    #plt.clf()  # Limpiar la figura para la siguiente

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
    # Obtener colores inicial y final desde el colormap
    start_color = colors[0]      # Color del punto inicial
    end_color = colors[-1]       # Color del punto final

    for i in range(len(x) - 1):
        #ax.plot(x[i:i+2], y[i:i+2], z[i:i+2], color=colors[i], linewidth=2, alpha=0.9)
        #ax.plot(x[i:i+2], z[i:i+2], y[i:i+2], color=colors[i], linewidth=2, alpha=0.9)  #tr
        ax.plot(z[i:i+2], x[i:i+2],y[i:i+2], color=colors[i], linewidth=2, alpha=0.9) #tt y tr

    # Ajustar etiquetas con espacio adicional para evitar corte
    ax.set_xlabel('X [m]', labelpad=5, fontsize=16, fontname='Times New Roman')
    ax.set_ylabel('Y [m]', labelpad=5, fontsize=16, fontname='Times New Roman')
    ax.set_zlabel('Z [m]', labelpad=5, fontsize=16, fontname='Times New Roman')
    # Aumentar tamaño de las marcas de los ejes
    #plt.tick_params(axis='both', which='major', labelsize=12)

    # Ajustar el espaciado de los ticks en 3D (tr)
    ax.set_yticks(np.arange(-0.1, 0.501, step=0.2)) #tr
    #ax.set_yticks(np.arange(-0.1, 0.501, step=0.1)) #tt
    ax.set_xticks(np.arange(-0.3,-0.7, step=-0.1))
    ax.set_zticks(np.arange(0.950, 1.351, step=0.1))

    # Cambiar la fuente de los ticks a Times New Roman y formatear para que muestren solo un decimal
    ax.set_xticklabels([f'{tick:.1f}' for tick in ax.get_xticks()], fontname="Times New Roman", fontsize=14)
    ax.set_yticklabels([f'{tick:.1f}' for tick in ax.get_yticks()], fontname="Times New Roman", fontsize=14)
    ax.set_zticklabels([f'{tick:.1f}' for tick in ax.get_zticks()], fontname="Times New Roman", fontsize=14)

    # Resaltar el punto inicial y final en la gráfica 3D con sus respectivos colores
    ax.scatter(z[0], x[0], y[0], color=start_color, s=100, edgecolors='black', label="Inicio")
    ax.scatter(z[-1], x[-1], y[-1], color=end_color, s=100, edgecolors='black', label="Final")

    # Ajustar el espaciado de los ticks en 3D (tt)
    #ax.set_yticks(np.arange(50, 501, step=100))
    #ax.set_xticks(np.arange(-651, -250, step=100))
    #ax.set_zticks(np.arange(950, 1351, step=100))

    # Ajustar vista para mejor percepción de la trayectoria
    ax.view_init(elev=30, azim=120)

    plt.show()

    # Guardar la figura 3D
    plt.savefig("3D.png", dpi=300, transparent=False)



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
