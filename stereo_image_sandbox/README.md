# stereo image sandbox

```
mkdir -p ~/ros/stereo_ws/src
wget https://raw.githubusercontent.com/iory/jsk_demos/stereo/stereo_image_sandbox/rosinstall -O ~/ros/stereo_ws/src/.rosinstall
cd ~/ros/stereo_ws
source /opt/ros/${ROS_DISTRO}/setup.bash
cd ~/ros/stereo_ws/src
wstool up
cd ~/ros/stereo_ws
rosdep update
rosdep install --from-paths -i -y -r .
catkin b stereo_image_sandbox -j4
```

## Install udev

```
sudo cp ./udev/99-uvc.rules /etc/udev/rules.d/99-uvc.rules
```

## Install for text-to-speech

```
pip3 install pydub gtts
```

## Install stereodemo for depth estimation

```
pip3 install pathlib
pip3 install torch
pip3 install git+https://github.com/nburrus/stereodemo
```
