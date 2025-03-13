import pandas as pd
import argparse
import os
import matplotlib.pyplot as plt

def load_data(csv_file):
    # Read the CSV file, skipping metadata rows
    data = pd.read_csv(csv_file, skiprows=6)

    # Check if the necessary columns exist
    required_columns = {'Time (Seconds)', 'qW', 'qX', 'qY', 'qZ'}
    if not required_columns.issubset(data.columns):
        print("The required quaternion columns are not present in the CSV file.")
        return None

    # Drop rows with NaN values in the important columns
    data = data.dropna(subset=required_columns)

    return data

def plot_quaternions(data):
    time = data['Time (Seconds)'].values
    qW = data['qW'].values
    qX = data['qX'].values
    qY = data['qY'].values
    qZ = data['qZ'].values

    # Aplicar estilo limpio
    plt.style.use("seaborn-v0_8-muted")
    
    plt.figure(figsize=(10, 6))
    plt.plot(time, qW, label='qW', color='blue', linewidth=2, alpha=0.8)
    plt.plot(time, qX, label='qX', color='red', linewidth=2, alpha=0.8)
    plt.plot(time, qY, label='qY', color='green', linewidth=2, alpha=0.8)
    plt.plot(time, qZ, label='qZ', color='purple', linewidth=2, alpha=0.8)

    plt.xlabel('Time (Seconds)', fontsize=14)
    plt.ylabel('Quaternion Value', fontsize=14)
    plt.title('Quaternion Components over Time', fontsize=16)
    plt.legend()
    plt.grid(True, linestyle='--', alpha=0.6)
    plt.tick_params(axis='both', which='major', labelsize=12)
    
    # Guardar la figura
    plt.savefig("quaternions.png", dpi=300, bbox_inches='tight', transparent=False)
    plt.show()

def main():
    parser = argparse.ArgumentParser(description="Plot quaternions from OptiTrack dataset")
    parser.add_argument('--input', required=True, help="Name of the dataset folder")
    args = parser.parse_args()

    # Path to the CSV file
    folder_path = f'/Volumes/SSD/archivos/KUKA_dev/environment_modeling/ROSBAGS/{args.input}/'
    csv_file = os.path.join(folder_path, 'groundtruth_raw.csv')

    # Load the data
    data = load_data(csv_file)
    if data is not None:
        plot_quaternions(data)

if __name__ == "__main__":
    main()
