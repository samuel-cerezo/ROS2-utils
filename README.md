
# ROS2 Utils – Data Acquisition and Processing Tools

This repository provides a collection of ROS 2 utilities and scripts developed during a research stay, aimed at supporting data collection, calibration, trajectory recording, and post-processing in robotics applications.

These tools were used in experiments involving real-time trajectory logging, coordinate transformation, depth data handling, and robot kinematics, providing a modular and reusable framework for research in robotic systems.

---

## 📁 Repository Structure

### `aditional-scripts/`
Utility scripts for handling motion capture data, robot kinematics, and trajectory generation. Includes tools for reading YAML files, processing MoCap poses, and replicating or saving trajectories.

- `driverTestFloatArray.py`: Test script for publishing/subscribing `Float32MultiArray` topics in ROS 2.
- `mocap2.py`, `read_MoCap_poses.py`: Process and parse MoCap poses.
- `replicateTrajectory.py`, `replicate_trajectory.sh`: Replicates or transforms existing trajectories.
- `save_trajectory.sh`: Bash script to record current robot trajectory.
- `reading_yaml.py`, `optical_calib_values.py`: Utility scripts to load calibration/config files.

---

### `calibration/`
Scripts and configuration files for camera and robot calibration, coordinate system definition, and transformation conversion.

- `camera_calibration.py`: Performs camera calibration routines.
- `coordinate_origin_sys.py`: Defines or shifts the coordinate frame origin.
- `fCSVtoT.py`: Converts CSV entries to transformation matrices.
- `robot-base.csv`, `transformations.yaml`: Stores static calibration values and reference poses.

---

### `create_trajectory/`
ROS 2 scripts that connect to the robot interface and log the joint states in real time, creating a trajectory dataset. These trajectories are typically used as reference motions or ground truth.

---

### `csv-verification/`
Scripts to verify the integrity and format of recorded trajectories (CSV). Checks include consistency in timestamps, completeness of pose data, and basic statistical sanity.

---

### `post_processing/`
Tools to convert, align, and visualize data after recording.

- `alignment_utils.py`: Aligns estimated and ground truth trajectories.
- `create_video.py`: Generates video outputs from recorded frames.
- `fFlange2world.py`, `fJoints2Pose.py`: Compute poses from joint or flange data.
- `reading_depth.py`, `show_depth.py`: Load and visualize depth images.
- `rosbag2TUM.py`: Converts recorded data to TUM trajectory format.
- `sync.py`: Synchronizes multiple data streams in time.

---

## ⚙️ Requirements

- ROS 2 Humble
- Python 3.8+
- Standard ROS 2 packages:
  - `rclpy`, `sensor_msgs`, `geometry_msgs`, etc.
- Optional dependencies:
  - `numpy`, `opencv-python`, `matplotlib`, `PyYAML`

Install Python dependencies via:

```bash
pip install -r requirements.txt
```

---

## 🚀 Usage Examples

```bash
# Launch the robot camera node
./start_camera_samuel.sh

# Record a data session
./record_data_samuel.sh

# Replicate a recorded trajectory
python aditional-scripts/replicateTrajectory.py --input traj.csv

# Verify CSV data integrity
python csv-verification/verify_csv.py --input traj.csv

# Convert rosbag to TUM format
python post_processing/rosbag2TUM.py --bag mydata.bag --output traj.tum
```

---

## 🧪 Example Workflow

1. Start camera and robot nodes.
2. Launch recording with `record_data_samuel.sh`.
3. Process and convert the trajectory using scripts in `post_processing/`.
4. Optionally validate the data with tools from `csv-verification/`.
5. Calibrate the system using tools in `calibration/`.

---

## 👤 Author

Developed by **Samuel Cerezo** during a robotics research stay in 2025.

---

## 📜 License

This code is released for academic and research use. For commercial applications or redistribution, please contact the author.
