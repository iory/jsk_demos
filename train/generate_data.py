import multiprocessing
import argparse
import datetime
import cv2
from tqdm import tqdm
from eos import run_many
from eos import makedirs
from pathlib import Path
from labelme_utils import convert_coco2yolo
from labelme_utils import labelme2coco
from labelme_utils import get_class_names_from_labelme_jsons
import subprocess
from data import download_yolo7_segmentation


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
    args = parser.parse_args()

    start_time = datetime.datetime.now()
    n = args.n
    batch_size = args.b


    outpath_base = (Path(args.from_images_dir) / 'generated_data').resolve()
    makedirs(outpath_base)
    with open(str(outpath_base / '.gitignore'), 'w') as f:
        f.write('*\n')

    if len(args.from_images_dir) > 0:
        from remove_bg import remove_background
        target = 'from_images_dir'
        paths = list(sorted(Path(args.from_images_dir).glob('*/*.jpg'))) \
            + list(sorted(Path(args.from_images_dir).glob('*/*.jpeg'))) \
            + list(sorted(Path(args.from_images_dir).glob('*/*.png')))
        rembg_outpath = outpath_base / 'preprocessing' / 'rembg'
        target_names = []
        print('Remove background from images')
        for path in tqdm(paths):
            try:
                makedirs(rembg_outpath / path.parent.name)
                out_img = remove_background(cv2.imread(str(path)))
                cv2.imwrite(str(rembg_outpath / path.parent.name / path.with_suffix('.png').name), out_img)
                target_names.append(path.parent.name)
            except Exception as e:
                print(str(e))
        target_names = sorted(list(set(target_names)))
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

    num = 1
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
    subprocess.call('cp {}/images/*.jpg {}/images/train/'.format(outpath_base, outpath_base),
                    shell=True)
    subprocess.call('cp {}/images/*.jpg {}/images/test/'.format(outpath_base, outpath_base),
                    shell=True)

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
