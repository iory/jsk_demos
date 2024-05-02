# Manip teaching device

rosrun camera_calibration cameracalibrator.py --size 7x10 --square 0.025 image:=/t265/fisheye2/image_raw --no-service-check

## Prerequisite

```
sudo apt-get install -y libspatialindex-dev freeglut3-dev libsuitesparse-dev libblas-dev liblapack-dev ros-noetic-position-controllers ros-noetic-joint-trajectory-controller ros-noetic-controller-manager ros-noetic-diff-drive-controller ros-noetic-teleop-twist-keyboard python3-vcstool
pip3 install -U pip setuptools
pip3 install scikit-robot cameramodels
mkdir -p ~/ros/manip_teaching/src
cd ~/ros/manip_teaching/src
wget https://raw.githubusercontent.com/iory/jsk_demos/manip-teaching/vcsinstall.noetic -O- | vcs import
cd ~/ros/manip_teaching
rosdep install --from-paths -i -y -r .
catkin build --cmake-args -DCMAKE_BUILD_TYPE=Release
source ~/ros/manip_teaching/devel/setup.bash
```

## Run

In radxa,

```
roslaunch manip_teaching device.launch
```

In client PC,

```
roslaunch manip_teaching republish_images.launch
```

### Pseudo vacuum publisher

To save image by pseudo vacuum trigger, you can use `pseudo_vacuum_publisher.py`.

```
rosrun manip_teaching pseudo_vacuum_publisher.py
```

This ROS node publishes alternating values of `0` and `1000` to the `/vacuum_pressure` topic each time the Enter key is pressed.
It is implemented using the rospy library in Python and is intended for use in robotics applications that require control of vacuum pressure based on user input.


