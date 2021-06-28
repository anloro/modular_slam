# modular_slam

Tested in Ubuntu 20.04! Some modifications needed to run on different OS.

## Necessary packages

### SFML
```
sudo apt-get install libsfml-dev
```

### GTSAM
Add PPA
```
sudo add-apt-repository ppa:borglab/gtsam-release-4.0
sudo apt update  # not necessary since Bionic
```
Install:
```
sudo apt install libgtsam-dev libgtsam-unstable-dev
```

### Other dependencies
```
Eigen v3.3
tbb
pthread 
cpp std17
```

## Installation
```bash
mkdir build
cd build/
cmake -DBUILD_SHARED_LIBS=ON ..
make 
sudo make install
```

## Bibliography
rtabmap  
rtabmap-ros  
apriltag models: https://github.com/koide3/gazebo_apriltag  

### Additional packages used during testing

### apriltag_ros
```
sudo apt-get install ros-noetic-apriltag-ros
```

### turtlebot3
```
sudo apt-get install ros-noetic-turtlebot3
sudo apt-get install ros-noetic-gazebo
sudo apt-get install ros-noetic-bringup
sudo apt-get install ros-noetic-turtlebot3-teleop
```