#!/usr/bin/env python3
import cv2
import threading
import os
import numpy as np
from Tools.DualCSI import CSI_Camera, gstreamer_pipeline
import glob
import pickle
import time

import orbslam2

width = 640
height = 360

if os.path.exists(f'Calibration/Calibleft-{height}.pkl') and os.path.exists(f'Calibration/Calibright-{height}.pkl'):
    tempon = pickle.load(open(f'Calibration/Calibleft-{height}.pkl', 'rb'))
    K_Left = tempon[0]
    D_Left = tempon[1]
    mapLeft1, mapLeft2 = cv2.fisheye.initUndistortRectifyMap(K_Left, D_Left, np.eye(3), K_Left, (width, height), cv2.CV_16SC2)    
    tempon = pickle.load(open(f'Calibration/Calibright-{height}.pkl', 'rb'))
    K_Right = tempon[0]
    D_Right = tempon[1]
    mapRight1, mapRight2 = cv2.fisheye.initUndistortRectifyMap(K_Right, D_Right, np.eye(3), K_Right, (width, height), cv2.CV_16SC2)
    K_Left, K_Right, D_Left, D_Right, tempon = None, None, None, None, None

Path = "Calibration"
list_Right = glob.glob(f'{Path}/Right/*.png')
list_Left = glob.glob(f'{Path}/Left/*.png')

index_img = len(list_Right)

left_camera = CSI_Camera()
left_camera.open(
    gstreamer_pipeline(
        sensor_id=0,
        sensor_mode=3,
        flip_method=0,
        display_height=height,
        display_width=width,
    )
)
left_camera.start()

right_camera = CSI_Camera()
right_camera.open(
    gstreamer_pipeline(
        sensor_id=1,
        sensor_mode=3,
        flip_method=0,
        display_height=height,
        display_width=width,
    )
)
right_camera.start()

vocab_path = "../Vocabulary/ORBvoc.txt"
settings_path = "../config/Stereo.yaml"

slam = orbslam2.System(vocab_path, settings_path, orbslam2.Sensor.STEREO)
slam.set_use_viewer(False)
slam.initialize()

start_dt = time.time()

while True:
    _ , left_image=left_camera.read()
    _ , right_image=right_camera.read()
    
    t1 = time.time()
    tframe = t1 - start_dt
    rotationPoseMatrix = slam.process_image_stereo(left_image, right_image, tframe*1000)
    state =  slam.get_tracking_state()
    print(state)

    orbframe = slam.get_frame()
    
    cv2.imshow("CSI Left", left_image)
    cv2.imshow("Orbframe", orbframe)

    keyCode = cv2.waitKey(30) & 0xFF
    # Stop the program on the ESC key
    if keyCode != 255:
        print(keyCode)   
    if keyCode == 27:
        break
    elif keyCode == 32:
        index_img += 1
        cv2.imwrite(f'{Path}/Left/left_cam-{index_img}.png', left_image)
        cv2.imwrite(f'{Path}/Right/right_cam-{index_img}.png', right_image)

left_camera.stop()
left_camera.release()
right_camera.stop()
right_camera.release()
cv2.destroyAllWindows()
