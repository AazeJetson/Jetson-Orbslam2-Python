import cv2
import orbslam2
import time
import threading
import os
import numpy as np
from Tools.DualCSI import CSI_Camera, gstreamer_pipeline

os.chdir("..")

currentfolder = os.getcwd()
mapname = "MapMonocular.map"

vocab_path = f"{currentfolder}/ORB-SLAM2-GPU-RGBD-PYTHON/Vocabulary/ORBvoc.txt"
settings_path = f"{currentfolder}/example/csi_cam.yaml"

slam = orbslam2.System(vocab_path, settings_path, orbslam2.Sensor.MONOCULAR)
# Load map neeb to be set between the constructor and the initializer
if os.path.isfile(f"{currentfolder}/example/{mapname}"):
    slam.LoadMap(f"{currentfolder}/example/{mapname}")
slam.set_use_viewer(True)
slam.initialize()

width = 640
height = 360

left_camera = CSI_Camera()
left_camera.open(
    gstreamer_pipeline(
        sensor_id=1,
        sensor_mode=5,
        flip_method=0,
        display_height=height,
        display_width=width,
    )
)
left_camera.start()

start_dt = time.time()

while True:
    _ , left_image=left_camera.read()

    t1 = time.time()
    tframe = t1 - start_dt
    rotationPoseMatrix = slam.process_image_mono(left_image, tframe*1000)
    state = slam.get_tracking_state()

    if state == orbslam2.TrackingState.OK:
        euler = slam.get_euler()
        pose = slam.get_pose()

    orbframe = slam.get_frame()

    cv2.imshow("CSI Left", left_image)
    cv2.imshow("Orbframe", orbframe)
    # This also acts as
    keyCode = cv2.waitKey(1) & 0xFF
    # Stop the program on the ESC key
    if keyCode != 255:
        print(keyCode)   
    if keyCode == 27:
        break
    elif keyCode == 32:
        print(f"{currentfolder}/{mapname}")
    elif keyCode == 115: # "s"
        slam.SaveMap(f"{currentfolder}/{mapname}")

left_camera.stop()
left_camera.release()
slam.shutdown()
cv2.destroyAllWindows()
