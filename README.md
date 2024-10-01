# d435

The idea of this repository is to save all the experiments that i am performing with camera/robot using ROS2 humble and Python

# File formats 

- The color images are stored as 640x480 8-bit RGB images in PNG format.
- The depth maps are stored as 640x480 16-bit monochrome images in PNG format.
- The color and depth images are already pre-registered, i.e., the pixels in the color and depth images correspond already 1:1.
- --(The depth images are scaled by a factor of 5000, i.e., a pixel value of 5000 in the depth image corresponds to a distance of 1 meter from the camera, 10000 to 2 meter distance, etc. A pixel value of 0 means missing value/no data.
