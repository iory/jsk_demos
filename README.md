# balloon_detection

Detect balloon hand-off (`RECEIVED`) and fly-away (`FLEW_AWAY`) events from
video using YOLO-WorldV2 + ByteTrack + a wall-clock state machine, exposed
as a ROS 1 node that subscribes to `sensor_msgs/Image`.

## Setup

Tested on ROS One (`/opt/ros/one`).

```bash
sudo apt install \
    ros-one-catkin-virtualenv \
    ros-one-video-stream-opencv \
    ros-one-cv-bridge \
    ros-one-image-view
```

Clone into a catkin workspace and build:

```bash
mkdir -p ~/ros/balloon-detection/src
cd ~/ros/balloon-detection/src
git clone --single-branch https://github.com/iory/jsk_demos -b balloon_detection

cd ~/ros/balloon-detection
source /opt/ros/one/setup.bash
catkin build
```

The first build creates a per-package virtualenv and installs `ultralytics`,
`lap`, and the OpenAI `clip` module into it. This pulls torch and takes a few
minutes.

## Run

```bash
source /opt/ros/one/setup.bash
source ~/ros/balloon-detection/devel/setup.bash
```

Two launch files are provided.

### `video.launch` — sample-video demo

Plays an mp4 through `video_stream_opencv` and feeds it into the detector.

```bash
roslaunch balloon_detection video.launch video:=IMG_8615.mp4
```

Drop your own `*.mp4` files into `pooh/` and pass the basename via `video:=`,
or specify an arbitrary path (or `/dev/video0`) via `video_path:=`.

| Arg | Default | Notes |
|---|---|---|
| `video` | `IMG_8615.mp4` | File name under `pooh/`. |
| `video_path` | `$(find balloon_detection)/pooh/$(arg video)` | Override with absolute path. |
| `publish_fps` | `30` | `video_stream_opencv` publish rate. |
| `loop` | `false` | Loop the video file. |
| `visualize` | `true` | Open `image_view` on the annotated output. |
| `device` | `cpu` | e.g. `cuda:0`. |
| `balloon_classes` | `balloon,red balloon` | CSV YOLO-World prompts for the balloon. |
| `teddy_classes` | `teddy bear,stuffed animal,Winnie the Pooh plush` | CSV prompts for the receiver. |

### `detect_events.launch` — detection only

Use this when you already have an image source (a real camera, a rosbag,
another launch). Pass the topic via `image_topic:=`:

```bash
roslaunch balloon_detection detect_events.launch \
    image_topic:=/my_camera/color/image_raw visualize:=true
```

Accepts the same `balloon_classes`, `teddy_classes`, `device`, and `model`
args as `video.launch`.

## Outputs

Topics:

- `/detect_events/image_annotated` (`sensor_msgs/Image`) — annotated frames
- `/detect_events/state` (`std_msgs/String`, latched) — current state name (one of `NO_BALLOON`, `APPROACHING`, `HELD`, `RELEASED`, `FLEW_AWAY`), republished only when it changes
- `/detect_events/events` (`std_msgs/String`) — one JSON line per event (state transitions and `RECEIVED`/`FLEW_AWAY` triggers)

To record the annotated stream or event log, use `rosbag record` on the
topics above.

## States and events

```
NO_BALLOON -> APPROACHING -> HELD --3s stable--> RECEIVED
                                \--lost / upward--> FLEW_AWAY --2s--> FLEW_AWAY
```

`RECEIVED` and `FLEW_AWAY` are the two events emitted on `~events`; everything
else is an internal state transition (also logged for debugging). All timing
is wall-clock (`rospy.Time`), so the state machine is robust to dropped frames.

## License

MIT. See [LICENSE](LICENSE).
