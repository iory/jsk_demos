# train from object images.

## prerequisite

```
sudo apt update
sudo apt install python3 python3-pip python3-venv
sudo apt install -y python3-pyqt5
sudo apt install -y qt5-default
sudo apt install -y python-is-python3
pip3 install -r requirements.txt --verbose
python3 data.py
```

## remove background

```
python3 remove_bg.py yamagata_items
```

## train object detection model

```
python3 generate_data.py --target-image-dir rembg_img/shootingstar-2023-05-08-14-13-45-148729-88999 -t fan rau ba25
```
