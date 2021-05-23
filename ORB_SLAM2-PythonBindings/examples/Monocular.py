
import cv2
import orbslam2
import time

def gstreamer_pipeline(
    capture_width=1280,
    capture_height=720,
    display_width=640,
    display_height=360,
    framerate=30,
    flip_method=0,
):
    return (
        "nvarguscamerasrc ! "
        "video/x-raw(memory:NVMM), "
        "width=(int)%d, height=(int)%d, "
        "format=(string)NV12, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )


def show_camera():
    # To flip the image, modify the flip_method parameter (0 and 2 are the most common)
    print(gstreamer_pipeline(flip_method=0))
    cap = cv2.VideoCapture(gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)

    vocab_path = "/home/nanobot/Downloads/ORB-SLAM2-GPU-RGBD-PYTHON/Vocabulary/ORBvoc.txt"
    settings_path = "/home/nanobot/Downloads/ORB-SLAM2-GPU-RGBD-PYTHON/gpu/csi_cam.yaml"

    slam = orbslam2.System(vocab_path, settings_path, orbslam2.Sensor.MONOCULAR)
    slam.set_use_viewer(False)
    slam.initialize()

    start_dt = time.time()

    if cap.isOpened():
        window_handle = cv2.namedWindow("CSI Camera", cv2.WINDOW_AUTOSIZE)
        # Window
        while cv2.getWindowProperty("CSI Camera", 0) >= 0:
            ret_val, img = cap.read()

            t1 = time.time()
            tframe = t1 - start_dt
            rotationPoseMatrix = slam.process_image_mono(img, tframe*1000)
            state = slam.get_tracking_state()

            if state = "OK":
                print(rotationPoseMatrix)

            orbframe = slam.get_frame()

            cv2.imshow("CSI Camera", img)
            cv2.imshow("Orbframe", orbframe)
            keyCode = cv2.waitKey(30) & 0xFF
            if keyCode == 27:
                break
        cap.release()
        cv2.destroyAllWindows()
    else:
        print("Unable to open camera")


if __name__ == "__main__":
    show_camera()