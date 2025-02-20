import pandas as pd
import argparse
import os
import matplotlib.pyplot as plt
import numpy as np

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
    if 'qW' in data.columns and 'qX' in data.columns and 'qY' in data.columns and 'qZ' in data.columns:
        for index, row in data.iterrows():
            norm = (row['qX']**2 + row['qY']**2 + row['qZ']**2 + row['qW']**2)**0.5
            if abs(norm - 1) > 0.01:  # 1% tolerance
                print(f"Warning: The quaternion in frame {row['Frame']} is not normalized.")

    else:
        print("No quaternions found in the file.")

def plot_trajectory(data, folder_path):
    # Extract positions X, Y, Z
    x = np.array(data['X'])
    y = np.array(data['Y'])
    z = np.array(data['Z'])

    # 2D Plot (in a new figure)
    fig2d = plt.figure(figsize=(8, 6))
    plt.plot(x, y, label='2D Trajectory (XY)')
    # Highlight the starting point
    plt.plot(x[0], y[0], 'go', label='Start')  # Start point (green)
    # Highlight the end point
    plt.plot(x[-1], y[-1], 'ro', label='End')  # End point (red)
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.legend()
    fig2d.suptitle('2D Trajectory', fontsize=14)  # Set the title of the window
    fig2d.tight_layout()
    # Save the 2D plot as a PNG
    fig2d.savefig(os.path.join(folder_path, 'trajectory_2d.png'))

    # 3D Plot (in a separate figure)
    fig3d = plt.figure(figsize=(8, 6))
    ax = fig3d.add_subplot(111, projection='3d')
    ax.plot(x, y, z, label='3D Trajectory')
    # Highlight the starting point
    ax.scatter(x[0], y[0], z[0], color='green', s=100, label='Start')  # Start point (green)
    # Highlight the end point
    ax.scatter(x[-1], y[-1], z[-1], color='red', s=100, label='End')  # End point (red)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    plt.legend()
    fig3d.suptitle('3D Trajectory', fontsize=14)  # Set the title of the window
    fig3d.tight_layout()
    # Save the 3D plot as a PNG
    fig3d.savefig(os.path.join(folder_path, 'trajectory_3d.png'))

    # Plot quaternion components (qX, qY, qZ, qW)
    if 'qX' in data.columns and 'qY' in data.columns and 'qZ' in data.columns and 'qW' in data.columns:
        frames = data['Frame'].values  # Convert to 1D array
        qX = data['qX'].values  # Convert to 1D array
        qY = data['qY'].values  # Convert to 1D array
        qZ = data['qZ'].values  # Convert to 1D array
        qW = data['qW'].values  # Convert to 1D array

        # Quaternion Components Plot (in a new figure)
        fig_quaternions = plt.figure(figsize=(8, 6))
        plt.plot(frames, qX, label='qX')
        plt.plot(frames, qY, label='qY')
        plt.plot(frames, qZ, label='qZ')
        plt.plot(frames, qW, label='qW')
        plt.xlabel('Frame')
        plt.ylabel('Component')
        plt.title('Quaternion Components vs Frames')
        plt.legend()

        # Add more ticks on the x-axis for better visibility
        tick_positions = np.arange(0, len(frames), step=500)
        plt.xticks(tick_positions, frames[tick_positions], rotation=45)  # Set the ticks with labels

        fig_quaternions.suptitle('Quaternion Components', fontsize=14)  # Set the title of the window
        fig_quaternions.tight_layout()
        # Save the quaternion plot as a PNG
        fig_quaternions.savefig(os.path.join(folder_path, 'quaternions.png'))

    plt.close('all')  # Close all figures to avoid memory overflow

def main():
    parser = argparse.ArgumentParser(description="Analysis of OptiTrack position and rotation data")
    parser.add_argument('--input', required=True, help="Name of the dataset folder")
    args = parser.parse_args()

    # Path to the CSV file
    folder_path = f'/home/samuel/dev/environment_modeling/ROSBAGS/{args.input}/'
    csv_file = os.path.join(folder_path, f'{args.input}.csv')

    # Load the data
    data = load_data(csv_file)
    if data is not None:
        # Check quaternions
        check_quaternions(data)

        # Plot the trajectories and orientations, and save the figures
        plot_trajectory(data, folder_path)

if __name__ == "__main__":
    main()
