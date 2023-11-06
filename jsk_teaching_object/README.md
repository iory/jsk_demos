# jsk teaching object package

This package provides a simple human teachable function for the robot to recognize objects.

## Install

```
mkdir -p ~/jsk_teaching_object/src
cd ~/jsk_teaching_object/src
wstool init
wstool merge https://raw.githubusercontent.com/iory/jsk_demos/teaching-object/jsk_teaching_object/noetic.rosinstall
wstool update
cd ../
source /opt/ros/noetic/setup.bash
rosdep update
rosdep install -y -r --from-paths src --ignore-src
catkin build jsk_teaching_object
source devel/setup.bash
```

### for r8 demo

```bash
mkdir -p ~/ros/r8/src/jsk-ros-pkg
cd  ~/ros/r8/src/jsk-ros-pkg
cd  ~/ros/r8/src/
cp ~/ros/r8/src/jsk-ros-pkg/jsk_demos/jsk_teaching_object/jsk_r8.rosinstall.noetic .rosinstall
wstool update -t .
source /opt/ros/$ROS_DISTRO/setup.bash
cd ~/ros/r8/src/jsk-ros-pkg/jsk_demos/jsk_teaching_object
rosdep install -y -r --from-paths . --ignore-src
cd ~/ros/r8/src/jsk-ros-pkg/jsk_demos/fg_ros
rosdep install -y -r --from-paths . --ignore-src
cd ~/ros/r8/src/seed-solutions
rosdep install -y -r --from-paths . --ignore-src
cd ../
catkin build jsk_teaching_object r8_5
source ~/ros/r8/devel/setup.bash
```

## Training

comming soon.

## Run trained models.

```
roslaunch jsk_teaching_object edgetpu_detection.launch INPUT_IMAGE:=/openni_camera/rgb/image_raw \
    model_file:=<MODEL_PATH> \
    label_file:=<LABEL_FILE_PATH>
```

## Run sample trained models.

```
roslaunch jsk_teaching_object sample_edgetpu_detection_with_depth_filter.launch
```

![](./doc/recognition.gif)

### for industry objects

```
roslaunch jsk_teaching_object sample_foreground_detection.launch
```
