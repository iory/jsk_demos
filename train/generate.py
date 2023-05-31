import argparse
from collections import defaultdict
from pathlib import Path

from eos import make_fancy_output_dir
from eos import makedirs
import numpy as np
from PIL import Image
from tqdm import tqdm

from aug import random_binpacking
from aug import random_crop_with_size
from data import download_bg_dataset
from data import download_items
from labelme_utils import create_instance_mask_json
from targets_info import *
from top_n_color import take_random_n_color


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='data generator')
    parser.add_argument('-n', default=1000, type=int)
    parser.add_argument('--image-width', default=300, type=int)
    parser.add_argument('--target', default='', type=str)
    parser.add_argument('--target-names', nargs='+')
    parser.add_argument('--out', '-o', default='./gen_data', type=str)
    parser.add_argument('--min-scale', default=None)
    parser.add_argument('--max-scale', default=None)
    parser.add_argument('--target-image-dir', type=str)
    args = parser.parse_args()

    max_angle = 360
    min_scale = 0.2
    max_scale = 0.6
    if len(args.target) > 0:
        targets, max_angle, min_scale, max_scale = get_targets(args.target)
    else:
        targets = args.target_names
    if args.min_scale is not None:
        min_scale = float(args.min_scale)
    if args.max_scale is not None:
        max_scale = float(args.max_scale)
    targets = [tgt.lower() for tgt in targets]

    dst_path = Path(make_fancy_output_dir(args.out, args=args, no_save=True))
    train_path = dst_path / 'train'
    test_path = dst_path / 'test'
    makedirs(train_path)
    makedirs(test_path)
    if len(args.target_image_dir) > 0:
        img_paths = list(sorted(Path(args.target_image_dir).glob('*/*.png')))
    else:
        img_paths = download_items()
    img_probs = np.zeros(len(img_paths))
    cnt_dict = defaultdict(int)
    for i, ip in enumerate(img_paths):
        obj_name = ip.parent.name.lower()
        if obj_name not in targets:
            cnt_dict['others'] += 1
        else:
            cnt_dict[obj_name] += 1
    not_others_cnt = len(img_paths) - cnt_dict['others']
    if cnt_dict['others'] == 0:
        others_p = 0.0
        p = 1.0
        p_per_class = p / (len(cnt_dict) - 1)
    else:
        p = 0.9
        others_p = (1 - p) / cnt_dict['others']
        p_per_class = p / (len(cnt_dict) - 1)

    for i, ip in enumerate(img_paths):
        obj_name = ip.parent.name.lower()
        if obj_name == 'others' or obj_name not in targets:
            img_probs[i] = others_p
        else:
            img_probs[i] = p_per_class / cnt_dict[obj_name]

    bg_paths = sorted((Path(download_bg_dataset()) / 'train').glob('*.jpg'))

    acc_cnt = defaultdict(int)
    for i, ip in enumerate(img_paths):
        obj_name = ip.parent.name.lower()
        if obj_name == 'others' or obj_name not in targets:
            acc_cnt['others'] = 0
        else:
            acc_cnt[obj_name] = 0

    class_names = sorted(list(cnt_dict.keys()))

    for i in tqdm(range(args.n)):
        if np.random.uniform(0, 1) < 0.5:
            # use real image
            bg_path = np.random.choice(bg_paths)
            pil_bg_img = Image.open(bg_path)
            bg_img = random_crop_with_size(pil_bg_img,
                                           image_width=args.image_width)
        else:
            # use synthetic texture
            tmp_path = np.random.choice(img_paths, p=img_probs)
            color = take_random_n_color(tmp_path, n=3)
            bg_img = np.ones(
                (args.image_width, args.image_width, 3), dtype=np.uint8) * np.array(color, dtype=np.uint8)
            bg_img = Image.fromarray(bg_img)
        gen_img, bboxes, names, instance_mask = random_binpacking(
            bg_img, img_paths, p=img_probs,
            targets=targets,
            image_width=args.image_width,
            max_angle=max_angle,
            min_scale=min_scale,
            max_scale=max_scale)
        for name in names:
            acc_cnt[name] += 1
        total = sum(acc_cnt.values())
        max_value = max(acc_cnt.values())
        probs = []
        new_names = []
        for key, value in acc_cnt.items():
            # probs.append(1.0 / (value + 1.0))
            probs.append((max_value - value) ** 2)
            new_names.append(key)
        probs = np.array(probs)
        if probs.sum() != 0.0:
            probs = probs / probs.sum()
        else:
            probs = 1 / len(probs) * np.ones(len(probs))
        name2prob = {nn: float(prob) for nn, prob in zip(new_names, probs)}

        for idx, ip in enumerate(img_paths):
            obj_name = ip.parent.name.lower()
            if obj_name == 'others' or obj_name not in targets:
                img_probs[idx] = name2prob['others'] / cnt_dict['others']
            else:
                img_probs[idx] = name2prob[obj_name] / cnt_dict[obj_name]

        img_path = train_path / f'{i:06}.jpg'
        # create_json(img_path, gen_img, bboxes, names)
        create_instance_mask_json(img_path, gen_img, instance_mask, names,
                                  class_names)

    class_names = ['_background_', ] + class_names
    with open(str(train_path / 'class_names.txt'), 'w') as f:
        f.write('\n'.join(class_names) + '\n')
