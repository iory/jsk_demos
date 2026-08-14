#!/usr/bin/env python
"""Train a YOLO pose model on the Ultralytics dog-pose dataset.

Ultralytics ships ``dog-pose.yaml`` as a *dataset* recipe, not as pretrained
weights, so the ROS node needs a checkpoint produced here first. The dataset
(337 MB, 6773 train / 1703 val images, 24 keypoints) is downloaded on the first
run into the Ultralytics datasets directory.

Examples
--------
::

    rosrun balloon_detection train_dog_pose.py --epochs 100 --device 0
    roscd balloon_detection && ls runs/dog-pose/weights/best.pt

Then point the node at the checkpoint::

    roslaunch balloon_detection dog_pose.launch \\
        model:=/abs/path/to/runs/dog-pose/weights/best.pt
"""

import argparse

from ultralytics import YOLO


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--model",
        default="yolo11n-pose.pt",
        help="Pretrained pose checkpoint to fine-tune from.",
    )
    parser.add_argument(
        "--data", default="dog-pose.yaml", help="Ultralytics dataset config."
    )
    parser.add_argument("--epochs", type=int, default=100)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--batch", type=int, default=16)
    parser.add_argument(
        "--device",
        default="",
        help="'0' for the first GPU, 'cpu' to force CPU, empty to autodetect.",
    )
    parser.add_argument("--project", default="runs", help="Output directory.")
    parser.add_argument("--name", default="dog-pose", help="Run name.")
    return parser.parse_args()


def main():
    args = parse_args()
    model = YOLO(args.model)
    results = model.train(
        data=args.data,
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        device=args.device or None,
        project=args.project,
        name=args.name,
    )
    print("best weights: {}/weights/best.pt".format(results.save_dir))


if __name__ == "__main__":
    main()
