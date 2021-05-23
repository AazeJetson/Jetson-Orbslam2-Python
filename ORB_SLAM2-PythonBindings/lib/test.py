import orbslam2
import numpy as np
import time
import cv2
import pyrealsense2 as rs
from scipy.spatial.transform import Rotation as R
import pcl

# import open3d as o3d
# from pyntcloud import PyntCloud

def main(vocab_path, settings_path):
    pipeline = rs.pipeline()
    config = rs.config()

    # Get device product line for setting a supporting resolution
    pipeline_wrapper = rs.pipeline_wrapper(pipeline)
    pipeline_profile = config.resolve(pipeline_wrapper)
    device = pipeline_profile.get_device()
    device_product_line = str(device.get_info(rs.camera_info.product_line))

    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    pipeline.start(config)
    
    slam = orbslam2.System(vocab_path, settings_path, orbslam2.Sensor.RGBD)
    slam.set_use_viewer(False)
    slam.initialize()

    start_dt = time.time()
    
    while True:
        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue

        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_BONE)

        cv2.imshow("jet", depth_colormap)

        t1 = time.time()
        tframe = start_dt - t1
        rotationPoseMatrix = slam.process_image_rgbd(color_image, depth_colormap, tframe*1000)
        t2 = time.time()

        rotationMatrix = rotationPoseMatrix[:3,:3]
        r = R.from_matrix(rotationMatrix)

        # pc=rs.pointcloud()
        # pc.map_to(depth_frame)
        # points = pc.calculate(depth_frame)
        # pcl_points=pcl.PointCloud()
        # point_to_pcl(pcl_points,points)

        # euler = r.as_euler('zyx', degrees=True)
        # euler = [euler[0], euler[2], euler[1]]
        # print(euler)

        # pose = np.squeeze(rotationPoseMatrix[:3,-1:], axis=1)
        # pose = [pose[2],pose[0],pose[1]]
        # print(pose)

        # quat = r.as_quat()
        # print(quat)

        # num_feat = slam.get_num_matched_features()
        # print(num_feat)

        mappts = slam.get_map_point()
        map = np.array(mappts)
        # print(mappts)

        refpts = slam.get_reference_point()
        mapref = np.array(refpts)
        # print(refpts)

        orbframe = slam.get_frame()
        
        # world_pos = slam.get_keyframe_points()
        # print(world_pos[len(world_pos)-1])

        # quat = slam.get_quaternion()
        # print(quat)

        # euler = slam.get_euler()
        # print(euler)

        # pose = slam.get_pose()
        # print(pose)

        cv2.imshow("frame", orbframe)
        key = cv2.waitKey(1)
        if key == ord("q"):
            break

        
    #save_trajectory(slam.get_trajectory_points(), 'trajectory.txt')

    slam.shutdown()

    # times_track = sorted(times_track)
    # total_time = sum(times_track)
    # print('-----')
    # print('median tracking time: {0}'.format(times_track[num_images // 2]))
    # print('mean tracking time: {0}'.format(total_time / num_images))

    return 0

if __name__ == '__main__':
    vocab = "/home/deskbot/Documents/cpp/My_orbslam/Vocabulary/ORBvoc.txt"
    param = "/home/deskbot/Documents/cpp/My_orbslam/config/D435.yaml"
    main(vocab_path=vocab, settings_path=param)