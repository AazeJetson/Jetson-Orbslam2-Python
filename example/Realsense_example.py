import pyrealsense2 as rs
import numpy as np
import cv2
import orbslam2
import os
import time

os.chdir("..")
path = os.getcwd()

mapname = "MapRGBD.map"
vocab_path = f"{path}/ORB-SLAM2-GPU-RGBD-PYTHON/Vocabulary/ORBvoc.txt"
settings_path = f"{path}/example/rgbd_real_sense.yaml"

pipeline = rs.pipeline()

config = rs.config()

config.enable_stream(rs.stream.depth, 640,480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640,480, rs.format.bgr8, 30)

profile = pipeline.start(config)
color_stream = profile.get_stream(rs.stream.color).as_video_stream_profile()
i = color_stream.get_intrinsics()
fx = i.fx/2
fy = i.fy/2
cx = i.ppx/2
cy = i.ppy/2
coeffs = i.coeffs

updatefile = []
with open(settings_path,"r") as f:
    lines = f.readlines()
    for line in lines:
        if "Camera.fx:" in line:
            line = f"Camera.fx: {fx}\n"
        elif "Camera.fy:" in line:
            line = f"Camera.fy: {fy}\n"
        elif "Camera.cx:" in line:
            line = f"Camera.cx: {cx}\n"
        elif "Camera.cy:" in line:
            line = f"Camera.cy: {cy}\n" 
        elif "Camera.k1:" in line:
            line = f"Camera.k1: {coeffs[0]}\n"
        elif "Camera.k2:" in line:
            line = f"Camera.k2: {coeffs[1]}\n"
        elif "Camera.p1:" in line:
            line = f"Camera.p1: {coeffs[2]}\n"
        elif "Camera.p2:" in line:
            line = f"Camera.p2: {coeffs[3]}\n"
        elif "Camera.k3:" in line:
            line = f"Camera.k3: {coeffs[4]}\n" 
        updatefile.append(line)

with open(settings_path, "w") as f:
    for line in updatefile:
        f.write(line)

slam = orbslam2.System(vocab_path, settings_path, orbslam2.Sensor.RGBD)

if os.path.isfile(f"{path}/example/{mapname}"):
    slam.LoadMap(f"{path}/example/{mapname}")

slam.set_use_viewer(True)
slam.initialize()

align_to = rs.stream.color
align = rs.align(align_to)

# Pour éviter un problème de luminosité au démarrage de la caméra
for i in range(10):
    frames = pipeline.wait_for_frames()
    aligned_frames = align.process(frames)

    depth_frame = aligned_frames.get_depth_frame()
    color_frame = aligned_frames.get_color_frame()

old_dt = time.time()

try:
    while True:
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)

        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()

        if not depth_frame or not color_frame:
            continue

        depth_image = np.asarray(depth_frame.get_data())
        color_image = np.asarray(color_frame.get_data())

        color_image = cv2.resize(color_image,(320,240))
        depth_image = cv2.resize(depth_image,(320,240))

        dt = time.time() - old_dt
        old_dt += dt

        rotationPoseMatrix = slam.process_image_rgbd(color_image, depth_image, dt*1000)
        state = slam.get_tracking_state()

        if state == orbslam2.TrackingState.OK:
            euler = slam.get_euler()
            pose = slam.get_pose()

        orbframe = slam.get_frame()

        cv2.imshow("depth", depth_image)
        cv2.imshow("color", color_image)
        cv2.imshow("orbframe", orbframe)
        key = cv2.waitKey(1)
        if key == ord("q") or key == 27:
            break
        elif key == 115: # "s"
            slam.SaveMap(f"{path}/example/{mapname}")

    cv2.destroyAllWindows()
finally:
    pipeline.stop()
