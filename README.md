# ORB-SLAM2-PYTHON-GPU-RGBD

This package combine modified libraries and script used to make orbslam2 cuda/rgbd easy to use with python.
This package was tested on fresh jetpack 4.5.1 install.

Each original license and name tag is kept in each package. If somethings was forgetten dont hesitate to advice me for making the expected change.

<a href="http://www.youtube.com/watch?feature=player_embedded&v=dF7_I2Lin54
" target="_blank"><img src="http://img.youtube.com/vi/dF7_I2Lin54/0.jpg"
alt="Tsukuba Dataset" width="240" height="180" border="10" /></a>
<a href="http://www.youtube.com/watch?feature=player_embedded&v=51NQvg5n-FE
" target="_blank"><img src="http://img.youtube.com/vi/51NQvg5n-FE/0.jpg"
alt="KITTI Dataset" width="240" height="180" border="10" /></a>
<a href="http://www.youtube.com/watch?feature=player_embedded&v=LnbAI-o7YHk
" target="_blank"><img src="http://img.youtube.com/vi/LnbAI-o7YHk/0.jpg"
alt="TUM RGBD Dataset" width="240" height="180" border="10" /></a>
<a href="http://www.youtube.com/watch?feature=player_embedded&v=MUyNOEICrf8
" target="_blank"><img src="http://img.youtube.com/vi/MUyNOEICrf8/0.jpg"
alt="EuRoC Dataset (V1_02, V1_03)" width="240" height="180" border="10" /></a>
<a href="http://www.youtube.com/watch?feature=player_embedded&v=xXt90wZejwk
" target="_blank"><img src="http://img.youtube.com/vi/xXt90wZejwk/0.jpg"
alt="EuRoC Dataset (V1_02, V1_03)" width="240" height="180" border="10" /></a>


## Installation

You need to install and compile some library before.
Each command begin at the root directories of this package of library.

For the next compile the system need a bigger swapfile.
```
cd resizeSwapMemory
./setSwapMemorySize.sh -g 6
```

Automatic compilation and installation of Opencv 4.5.1 with all contrib module (not free included) and cuda support.
If you want to change the version of opencv you have just to modifie the begining of script.
This is a modified version of a script from JetsonHacks.
```
cd buildOpenCV
./buildOpencv.sh
```

Automatic compilation of librealsense with python binding and cuda support
If you want the library to use cude : change the script buildLibrealsense.sh at line 13 from USE_CUDA=true to USE_CUDA=false 
Like the script above this one come from JetsonHacks too.
```
cd librealsense
./buildlibrealsense
```

Compilation of pangolin (3d opengl viewer used by orbslam) from https://github.com/uiop/pangolin
I don't remenber if i modified it for the purpose of this orbslam2 python binding (i think not).
```
cd pangolin
mkdir build && cd build
cmake .. && make -j3
cd ..
sudo python3 setup.py install
```

Compilation of Thridparty library used by orbslam2
Compilation des librairies utilisé par ORBslam2

DBOW2
```
cd ORB-SLAM2-GPU-RGBD-PYTHON/Thirdparty/DBoW2
mkdir build && cd build
cmake .. && make -j3
```

g2o
```
cd ORB-SLAM2-GPU-RGBD-PYTHON/Thirdparty/g2o
sudo apt-get install libblas-dev liblapack-dev
mkdir build && cd build
cmake .. && make -j3
```

Compile and installation of orbslam2
```
cd ORB-SLAM2-GPU-RGBD-PYTHON
sudo apt-get install libboost-all-dev
mkdir build && cd build
cmake .. && make -j3
sudo make install
```

before installing the python binding we nned to add one line in the bashrc
```
gedit ~/.bashrc
At the end of the file add :
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
```

then:
```
source ~/.bashrc
```

installing pip and pybind11
```
sudo apt-get install python3-pip
pip3 install "pybind11[global]"
```

then finally install the python binding of orbslam2
```
cd ORB_SLAM2-PythonBindings
mkdir build && cd build
cmake .. && make -j3
sudo make install
```

Now if everything worked you'll be able of importing the librarie under python3
You need to import cv2 before orbslam2 to avoid the library to crash.

## Usage

Before everything you need to know where is 2 file:
ORBvoc.txt : you can find a compressed version in ORB-SLAM2-GPU-RGBD-PYTHON/Vocabulary/ORBvoc.txt.tar.gz
and a yaml setting file like the 2 in the example directory
csi_cam.yaml : for the monocular version
rgbd_real_sense.yaml : for the rgbd version

### Basic python script example:

```
import cv2
import orbslam2
import time
import os

os.chdir("..")
currentfolder = os.getcwd()

vocab_path = f"{currentfolder}/ORB-SLAM2-GPU-RGBD-PYTHON/Vocabulary/ORBvoc.txt"
settings_path = f"{currentfolder}/example/csi_cam.yaml"

slam = orbslam2.System(vocab_path, settings_path, orbslam2.Sensor.MONOCULAR)

cap = cv2.VideoCapture(0)

slam.set_use_viewer(True)
slam.initialize()

offsetdt = time.time()

while True:
    _, img = cap.read()
    dt = time.time() - offsetdt
    offsetdt += dt

    rotationPoseMatrix = slam.process_image_mono(img, dt*1000)

    if slam.get_tracking_state() == orbslam2.TrackingState.OK:
        euler = slam.get_euler()
        pose = slam.get_pose()
    
    orbframe = slam.get_frame()

    cv2.imshow("CSI camera", img)
    cv2.imshow("Orbslam frame", orbframe)

    keyCode = cv2.waitKey(1) & 0xFF
 
    if keyCode == 27:
        break

slam.shutdown()
cv2.destroyAllWindows()
```

The library work well (20 fps with 640x480 frame) in monocular and rgbd but in stereo the performanc is really poor.

In the 2 example script you have everything for a basic usage of the library.
 
## To do

	add function to switch between localization only and mapping state. 

## list of function

### System :

Define the type of the process with the settings file, vocabulary and the type of sensor

	orbslam2.System(vocfile, settingfile, orbslam2.Sensor.MONOCULAR or orbslam2.Sensor.STEREO or orbslam2.Sensor.RGBD)

### initialize : 
	
	slam.initialize()

### process_image_mono :

	function used to feed the process with monocular frame

	the function return a rotation-translation matrix (3x4)

	where the 3 first cols is a rotation matrix (3x3) who represente the orientation and the last cols is the coordinates in his own reference as the monocular can't calcute well the distance.

### process_image_stereo :

    function used to feed the process with 2 frame from a stereo camera

    the function return a rotation-translation matrix (3x4)

    where the 3 first cols is a rotation matrix (3x3) who represente the orientation and the last cols is the world coordinate.

### process_image_rgbd :

    function used to feed the process with a frame and it depth frame.

    the function return a rotation-translation matrix (3x4)

    where the 3 first cols is a rotation matrix (3x3) who represente the orientation and the last cols is the world coordinates.

### shutdown :

    function used to close cleanely the process

### is_running :

    function who return a boolean if the process is running or not.

### reset :

    function for reseting the map

### set_use_viewer :

    funtion used to set if the viewer will be shown or not (if set to false you'll save some ressource)

### get_keyframe_points :

    function to get the cloud of point used in the current frame

### get_quaternion :

    return the orientation in form of quaternion 

### get_euler :

    return the orientation in radians

### get_pose :

    return pose coordinate.

### get_tracked_mappoints :

    return mappoints who are preloaded for the slam process

### get_tracking_state :

    return state of the process.

        orblsam2.TrackingState.SYSTEM_NOT_READY

        orblsam2.TrackingState.NO_IMAGES_YET

        orblsam2.TrackingState.NOT_INITIALIZED

        orblsam2.TrackingState.OK

        orblsam2.TrackingState.LOST


### get_map_point :

    return all map point

### get_reference_point :

    return reference point

### get_num_features :

    return the number of features find in the frame

### get_num_matched_features :

    return the number of features who match with the reference frame

### SaveMap :

    function for saving the map

        orbslam2.SaveMap("path_of_file/namefile.map")

### LoadMap :

    function for preloading map after the system was set and before the process is initialized 
	orbslam2.LoadMap("path_of_the_file/namefile.map")

###get_frame :

    return the frame updated with all information from the slam process (features, state etc.)
