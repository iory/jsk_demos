# KXR Cube solver


```
sudo apt-get install -y python3-vcstool
mkdir -p ~/ros/kxr_cube_solver/src
cd ~/ros/kxr_cube_solver/src
wget https://raw.githubusercontent.com/iory/jsk_demos/cube-solver/vcsinstall.noetic.yaml -O- | vcs import
cd ~/ros/kxr_cube_solver
rosdep install --from-paths -i -y -r .
catkin build --cmake-args -DCMAKE_BUILD_TYPE=Release
source ~/ros/kxr_cube_solver/devel/setup.bash
catkin b realsense2_camera --cmake-args -DCMAKE_BUILD_TYPE=Release
pip3 install pydub gtts
```
