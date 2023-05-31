import argparse
import datetime
import multiprocessing
from pathlib import Path
import shutil
import subprocess

import cv2
from eos import makedirs
from eos import run_many
import numpy as np
from pybsc.image_utils import add_alpha_channel
from pybsc.image_utils import apply_mask
from pybsc.image_utils import create_tile_image
from pybsc.image_utils import imread
from pybsc.image_utils import rotate
import six
from tqdm import tqdm

from data import download_yolo7_segmentation
from labelme_utils import convert_coco2yolo
from labelme_utils import get_class_names_from_labelme_jsons
from labelme_utils import labelme2coco


def run_command(cmd, *args, **kwargs):
    if kwargs.pop("capture_output", False):
        kwargs["stdout"] = subprocess.PIPE
        kwargs["stderr"] = subprocess.PIPE
    if six.PY2:
        return subprocess.check_call(cmd, *args, **kwargs)
    else:
        return subprocess.run(cmd, *args, **kwargs)



if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='data generator')
    parser.add_argument('-n', default=1000, type=int)
    parser.add_argument('-b', default=16, type=int)
    parser.add_argument('--target', default='fruit', type=str)
    parser.add_argument('-t', '--target-names', nargs='+')
    parser.add_argument('--no-train', action='store_true')
    parser.add_argument('--min-scale', default=None)
    parser.add_argument('--max-scale', default=None)
    parser.add_argument('--target-image-dir', type=str)
    parser.add_argument('--from-images-dir', type=str, default='')
    parser.add_argument('--compress-annotation-data', action='store_true')
    args = parser.parse_args()

    start_time = datetime.datetime.now()
    n = args.n
    batch_size = args.b


    outpath_base = (Path(args.from_images_dir) / 'generated_data').resolve()
    subprocess.call('rm -rf {}'.format(outpath_base),
                    shell=True)
    makedirs(outpath_base)
    with open(str(outpath_base / '.gitignore'), 'w') as f:
        f.write('*\n')

    if len(args.from_images_dir) > 0:
        from remove_bg import remove_background

        patterns = ["*.jpg", "*.JPG", "*.jpeg", "*.JPEG", "*.png", "*.PNG"]
        target = 'from_images_dir'
        paths = []
        for pattern in patterns:
            paths.extend(list(sorted(Path(args.from_images_dir).glob('*/{}'.format(pattern)))))
        rembg_outpath = outpath_base / 'preprocessing' / 'rembg'
        rembg_org_img_outpath = outpath_base / 'preprocessing' / 'rembg_org'
        img_and_rembg_outpath = outpath_base / 'preprocessing' / 'img_and_rembg'
        target_names = []
        print('Remove background from images')
        for path in tqdm(paths):
            try:
                makedirs(rembg_outpath / path.parent.name)
                makedirs(rembg_org_img_outpath / path.parent.name)
                makedirs(img_and_rembg_outpath / path.parent.name)
                org_img = imread(str(path), color_type='bgra', clear_alpha=True)
                out_img, (x1, y1, x2, y2), angle, mask = remove_background(
                    org_img.copy(), return_info=True)
                cv2.imwrite(str(rembg_outpath / path.parent.name / path.with_suffix('.png').name), out_img)
                cv2.imwrite(
                    str(rembg_org_img_outpath / path.parent.name / path.with_suffix('.jpg').name),
                    rotate(org_img[y1:y2, x1:x2], angle=angle))

                rembg_org_size_img = org_img.copy()
                rembg_org_size_img = apply_mask(rembg_org_size_img, mask, use_alpha=True)
                if org_img.shape[2] == 3:
                    concatenated_images = np.concatenate(
                        (add_alpha_channel(org_img, alpha=255), rembg_org_size_img), axis=1)
                else:
                    concatenated_images = np.concatenate((org_img, rembg_org_size_img), axis=1)
                cv2.imwrite(str(img_and_rembg_outpath / path.parent.name / path.with_suffix('.png').name),
                            concatenated_images)
                target_names.append(path.parent.name)
            except Exception as e:
                print(str(e))
        target_names = sorted(list(set(target_names)))
        # create tile image for summary.
        tile_image_outpath = outpath_base / 'preprocessing' / 'tile_rembg'
        makedirs(tile_image_outpath)
        for target_name in target_names:
            tile_img = create_tile_image(
                list((rembg_outpath / target_name).glob('*.png')),
                num_tiles_per_row=5)
            tile_img.save(tile_image_outpath / '{}.png'.format(target_name))

        tile_org_and_rembg_outpath = outpath_base / 'preprocessing' / 'tile_org_and_rembg'
        makedirs(tile_org_and_rembg_outpath)
        for target_name in target_names:
            tile_org_and_rembg_img_paths = []
            for a in (rembg_outpath / target_name).glob('*.png'):
                tile_org_and_rembg_img_paths.append(
                    rembg_org_img_outpath / target_name / a.with_suffix('.jpg').name)
                tile_org_and_rembg_img_paths.append(a)
            tile_img = create_tile_image(
                tile_org_and_rembg_img_paths, num_tiles_per_row=6)
            tile_img.save(tile_org_and_rembg_outpath / '{}.png'.format(target_name))
        cmd = 'python generate.py --out {} -n {} --image-width 300 --target-names {}'.format(
            outpath_base, n,
            ' '.join(target_names))
        cmd += ' --target-image-dir {}'.format(rembg_outpath)
    else:
        # cmd = 'python generate.py --out /tmp/fg -n 1000 --target foreground'
        if args.target_names is None or len(args.target_names) == 0:
            target = args.target
            cmd = 'python generate.py --out {} -n {} --image-width 300 --target {}'.format(
                outpath_base, n, target)
        else:
            target = 'yamagata'
            cmd = 'python generate.py --out {} -n {} --image-width 300 --target-names {}'.format(
                outpath_base, n,
                ' '.join(args.target_names))
        if len(args.target_image_dir) > 0:
            cmd += ' --target-image-dir {}'.format(args.target_image_dir)

    if args.min_scale is not None:
        cmd += ' --min-scale {}'.format(args.min_scale)
    if args.max_scale is not None:
        cmd += ' --max-scale {}'.format(args.max_scale)

    num = 20
    jobs = min(multiprocessing.cpu_count(), num)
    sleep_time = 1.0
    verbose = False
    run_many(cmd, num, jobs=jobs,
             sleep_time=sleep_time, verbose=verbose)

    labelme_jsons = list(outpath_base.glob('*/*/*.json'))
    save_train_json_path = outpath_base / 'train.json'
    save_val_json_path = outpath_base / 'test.json'
    labelme2coco(
            labelme_jsons,
            save_train_json_path,
            save_val_json_path,
            split_train_val=True)
    convert_coco2yolo([save_train_json_path,
                       save_val_json_path],
                      outpath_base)

    for path in outpath_base.glob('images/*.jpg'):
        shutil.copy(path, outpath_base / 'images' / 'train' / path.name)
    for path in outpath_base.glob('images/*.jpg'):
        shutil.copy(path, outpath_base / 'images' / 'test' / path.name)

    class_names = get_class_names_from_labelme_jsons(labelme_jsons)
    with open('{}/images/test/class_names.txt'.format(outpath_base), 'w') as f:
        f.write('__ignore__\n_background_\n' + '\n'.join(class_names) + '\n')
    with open('{}/images/train/class_names.txt'.format(outpath_base), 'w') as f:
        f.write('__ignore__\n_background_\n' + '\n'.join(class_names) + '\n')

    with open('{}/{}.yaml'.format(outpath_base, target), 'w') as f:
        f.write("""# COCO 2017 dataset http://cocodataset.org
# train and val data as 1) directory: path/images/, 2) file: path/images.txt, or 3) list: [path1/images/, path2/images/]
train: {}/train.txt
val: {}/test.txt
test: {}/test.txt
# number of classes
nc: {}
# class names
names: [{}]""".format(outpath_base, outpath_base, outpath_base,
                   len(class_names),
                   ", ".join(map(lambda x: "'{}'".format(x), class_names))))
        f.write('\n')

    if args.compress_annotation_data:
        print('compress annotation data')
        compress_cmd = 'find {} -maxdepth 1 -type f \\( -name "*.jpg" -o -name "*.json" \\) -print | tar -czvf {}/generated_data.tar.gz -T -'.format(
            outpath_base / 'images',
            outpath_base)
        run_command(compress_cmd, shell=True, capture_output=True)

    if args.no_train is False:
        create_venv = False
        yolo7_dir = download_yolo7_segmentation(create_venv=create_venv)
        python_exe = 'python'
        if create_venv:
            python_exe = "{}/python".format(yolo7_dir)
        train_cmd = '{} {}/segment/train.py --noval --noplots --data {}/{}.yaml --batch {} --weights "{}/yolov7-seg.pt" --cfg {}/models/segment/yolov7-seg.yaml --epochs 10 --name yolov7-seg-coco --img 300 --hyp hyp.scratch-high.yaml --project {} --noval --nosave'.format(
            python_exe, yolo7_dir, outpath_base, target, batch_size, yolo7_dir, yolo7_dir, outpath_base)
        subprocess.call(train_cmd, shell=True)
    end_time = datetime.datetime.now()
    print(end_time - start_time)
