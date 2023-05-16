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

## train object detection model

Run the train script. Specify the previous directory as an argument.

Images are sent to a remote server and object detection training is performed.

```
python train.py yamagata_items
```

After training, a file containing the trained model (`best-%Y-%m-%d-%H-%M-%S-%f.pt`) and class name information (`from_images_dir-%Y-%m-%d-%H-%M-%S-%f.yaml`) will be copied.
