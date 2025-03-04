import os
import argparse
import numpy as np

def load_data(file_path):
    data = []
    with open(file_path, 'r') as f:
        for line in f:
            if line.startswith('#'):
                continue
            values = list(map(float, line.strip().split()))
            data.append(values)
    return np.array(data)

def find_nearest_entry(data, timestamp, max_diff=0.05):
    time_diffs = np.abs(data[:, 0] - timestamp)
    min_idx = np.argmin(time_diffs)
    if time_diffs[min_idx] <= max_diff:
        return data[min_idx]
    return None

def align_data(dataset_path, input_file, output_file, max_time_diff=0.05):
    input_path = os.path.join(dataset_path, input_file)
    output_path = os.path.join(dataset_path, output_file)
    
    if not os.path.exists(input_path):
        print(f"Error: {input_file} not found.")
        return
    
    data = load_data(input_path)
    return data, output_path

def process_alignment(dataset_path, max_time_diff=0.05):
    associations_path = os.path.join(dataset_path, 'associations.txt')
    groundtruth_data, groundtruth_output = align_data(dataset_path, 'groundtruth.txt', 'groundtruth_aligned.txt', max_time_diff)
    flange_data, flange_output = align_data(os.path.join(dataset_path, 'robot_data'), 'flange_poses.txt', 'flange_poses_aligned.txt', max_time_diff)
    
    if groundtruth_data is None or flange_data is None:
        return
    
    with open(associations_path, 'r') as assoc_file, \
         open(groundtruth_output, 'w') as gt_output_file, \
         open(flange_output, 'w') as flange_output_file:
        
        gt_output_file.write("#timestamp qx qy qz qw tx ty tz\n")
        flange_output_file.write("#timestamp qx qy qz qw tx ty tz\n")
        
        for line in assoc_file:
            if line.startswith('#'):
                continue
            parts = line.strip().split()
            if len(parts) < 2:
                continue
            
            timestamp = float(parts[0])
            
            nearest_gt = find_nearest_entry(groundtruth_data, timestamp, max_time_diff)
            nearest_flange = find_nearest_entry(flange_data, timestamp, max_time_diff)
            
            if nearest_gt is not None:
                gt_output_file.write(f"{nearest_gt[0]:.9f} " + " ".join(map(str, nearest_gt[1:])) + "\n")
            else:
                print(f"Warning: No matching groundtruth for timestamp {timestamp:.9f}")
            
            if nearest_flange is not None:
                flange_output_file.write(f"{nearest_flange[0]:.9f} " + " ".join(map(str, nearest_flange[1:])) + "\n")
            else:
                print(f"Warning: No matching flange pose for timestamp {timestamp:.9f}")

def main():
    parser = argparse.ArgumentParser(description='Align groundtruth and flange poses to image timestamps')
    parser.add_argument('--input', type=str, required=True, help='Folder containing dataset')
    args = parser.parse_args()
    
    #dataset_path = os.path.join('/home/samuel/dev/environment_modeling/ROSBAGS', args.input)
    dataset_path = os.path.join('/Volumes/SSD/archivos/KUKA_dev/environment_modeling/ROSBAGS', args.input)

    process_alignment(dataset_path)
    
if __name__ == '__main__':
    main()

