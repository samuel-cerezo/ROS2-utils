import cv2
import os
import numpy as np


# ----------------- folders managing ------------------
rosfile_name = 'd435_test1'
file_path = "/home/samuel/Desktop/"

rgb_path = file_path + rosfile_name + '_data' + '/rgb'
depth_path = file_path + rosfile_name + '_data' + '/depth'

img = cv2.imread(depth_path + '/1728286188895709757.png', cv2.IMREAD_UNCHANGED)
#print(img.max())
sampleArray = np.array(img) 

float_array = np.array(img, dtype=float)
float_array = float_array*0.001
print('Depth values in meters:')
print(float_array)