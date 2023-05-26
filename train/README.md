# train from object images.

## prepare object images.

Take an image of the object and create a separate directory for each name.
Please refer to the directory `yamagata_items`.

```
$ tree yamagata_items/
yamagata_items/
├── ba25
│   ├── IMG_9596.HEIC.jpg
│   ├── IMG_9597.HEIC.jpg
│   └── IMG_9598.HEIC.jpg
├── fan
│   ├── IMG_9562.HEIC.jpg
│   ├── IMG_9563.HEIC.jpg
│   ├── IMG_9564.HEIC.jpg
│   └── IMG_9565.HEIC.jpg
└── rau
    ├── IMG_9616.HEIC.jpg
    ├── IMG_9617.HEIC.jpg
    └── IMG_9618.HEIC.jpg

```

## Server Access and Execution of train.py via SSH

In order to execute the train.py script on the server, it is necessary to establish an SSH connection.
We kindly request the server administrator to provide us with the SSH public key for secure access.


## train object detection model

Run the train script. Specify the previous directory as an argument.

Images are sent to a remote server and object detection training is performed.

```
python train.py yamagata_items
```

If you want to specify the ssh identify file, specify it with the `-i` option.

```
python -- train.py -i ~/.ssh/id_rsa_target yamagata_items
```

After training, a file containing the trained model (`<IMAGE_DIRECTORY_NAME>-%Y-%m-%d-%H-%M-%S-%f.pt`),
class name information (`<IMAGE_DIRECTORY_NAME>-%Y-%m-%d-%H-%M-%S-%f.yaml`),
removed background images (`<IMAGE_DIRECTORY_NAME>-%Y-%m-%d-%H-%M-%S-%f-preprocessing`)
and generated data (`<IMAGE_DIRECTORY_NAME>-%Y-%m-%d-%H-%M-%S-%f-generated_data.tar.gz`) will be copied.


## Execution in Local Environment (optional)

If you wish to execute the code in your local environment, please follow the instructions below:

0. Install nvidia docker

Please see the following site and install nvidia docker.

https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#docker

1. Build Docker Image:

Execute the `build.sh` script to build the Docker image. This script will handle the necessary dependencies and configurations. You can run the command as follows:

```
./build.sh
```

2. Run the Docker Image:

Once the image is successfully built, you can execute the `run.sh` script, providing the `TARGET_DIRECTORY` where you want the generated data and trained models to be stored. Use the following command:

```
./run.sh TARGET_DIRECTORY
```

After running the command, the script will generate the required data and store the trained models in the `TARGET_DIRECTORY/generated_data` directory.


## Run for ROS

### Install

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

### Running a Model Trained with Webcam

To execute a model trained with webcam, follow the steps below:

Set the model_path parameter to the path of the trained model. In the example provided, it is set to $(pwd)/best.pt. Make sure to replace this with the actual path to your trained model.
Specify the class_names parameter as a list of class names enclosed in square brackets.
In the given example, the classes are ['ba25', 'fan', 'rau']. Modify this list according to the classes in your trained model (see, `from_images_dir-%Y-%m-%d-%H-%M-%S-%f.yaml`.)

Here's an example command:

```
roslaunch jsk_teaching_object sample_foreground_detection_webcam.launch model_path:=$(pwd)/best.pt class_names:="['ba25', 'fan', 'rau']"
```
