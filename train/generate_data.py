import multiprocessing
import argparse
import datetime
import cv2
from tqdm import tqdm
from eos import run_many
from eos import makedirs
from eos import make_fancy_output_dir
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

    if len(args.from_images_dir) > 0:
        from remove_bg import remove_background
        target = 'from_images_dir'
        paths = list(sorted(Path(args.from_images_dir).glob('*/*.jpg'))) \
            + list(sorted(Path(args.from_images_dir).glob('*/*.jpeg'))) \
            + list(sorted(Path(args.from_images_dir).glob('*/*.png')))
        outpath = Path(make_fancy_output_dir('./rembg_img', no_save=True))
        target_names = []
        print('Remove background from images')
        for path in tqdm(paths):
            try:
                makedirs(outpath / path.parent.name)
                out_img = remove_background(cv2.imread(str(path)))
                cv2.imwrite(str(outpath / path.parent.name / path.with_suffix('.png').name), out_img)
                target_names.append(path.parent)
            except Exception as e:
                print(str(e))
        target_names = sorted(list(set(target_names)))
        cmd = 'python generate.py --out /tmp/{} -n {} --image-width 300 --target-names {}'.format(
            target, n,
            ' '.join(target_names))
        cmd += ' --target-image-dir {}'.format(outpath)
    else:
        # cmd = 'python generate.py --out /tmp/fg -n 1000 --target foreground'
        if args.target_names is None or len(args.target_names) == 0:
            target = args.target
            cmd = 'python generate.py --out /tmp/{} -n {} --image-width 300 --target {}'.format(target, n, target)
        else:
            target = 'yamagata'
            cmd = 'python generate.py --out /tmp/{} -n {} --image-width 300 --target-names {}'.format(
                target, n,
                ' '.join(args.target_names))
        if len(args.target_image_dir) > 0:
            cmd += ' --target-image-dir {}'.format(args.target_image_dir)

    if args.min_scale is not None:
        cmd += ' --min-scale {}'.format(args.min_scale)
    if args.max_scale is not None:
        cmd += ' --max-scale {}'.format(args.max_scale)
    subprocess.call('rm -rf /tmp/{}'.format(target),
                    shell=True)
    num = 20
    jobs = min(multiprocessing.cpu_count(), num)
    sleep_time = 1.0
    verbose = False
    run_many(cmd, num, jobs=jobs,
             sleep_time=sleep_time, verbose=verbose)


    labelme_jsons = list(Path('/tmp/{}'.format(target)).glob('*/*/*.json'))
    save_train_json_path = '/tmp/{}/train.json'.format(target)
    save_val_json_path = '/tmp/{}/test.json'.format(target)
    dst_path = '/tmp/{}'.format(target)
    labelme2coco(
            labelme_jsons,
            save_train_json_path,
            save_val_json_path,
            split_train_val=True)
    convert_coco2yolo([save_train_json_path,
                       save_val_json_path],
                      dst_path)
    subprocess.call('cp /tmp/{}/images/*.jpg /tmp/{}/images/train/'.format(target, target),
                    shell=True)
    subprocess.call('cp /tmp/{}/images/*.jpg /tmp/{}/images/test/'.format(target, target),
                    shell=True)

    class_names = get_class_names_from_labelme_jsons(labelme_jsons)
    with open('/tmp/{}/images/test/class_names.txt'.format(target), 'w') as f:
        f.write('__ignore__\n_background_\n' + '\n'.join(class_names) + '\n')
    with open('/tmp/{}/images/train/class_names.txt'.format(target), 'w') as f:
        f.write('__ignore__\n_background_\n' + '\n'.join(class_names) + '\n')

    with open('/tmp/{}/{}.yaml'.format(target, target), 'w') as f:
        f.write("""# COCO 2017 dataset http://cocodataset.org
    # train and val data as 1) directory: path/images/, 2) file: path/images.txt, or 3) list: [path1/images/, path2/images/]
    train: /tmp/{}/train.txt
    val: /tmp/{}/test.txt
    test: /tmp/{}/test.txt
    # number of classes
    nc: {}
    # class names
    names: [{}]""".format(target, target, target,
                   len(class_names),
                   ", ".join(map(lambda x: "'{}'".format(x), class_names))))
        f.write('\n')


    if args.no_train is False:
        yolo7_dir = download_yolo7_segmentation()
        train_cmd = '{}/yolov7seg/bin/python {}/segment/train.py --noval --noplots --data /tmp/{}/{}.yaml --batch {} --weights "{}/yolov7-seg.pt" --cfg {}/models/segment/yolov7-seg.yaml --epochs 10 --name yolov7-seg-coco --img 300 --hyp hyp.scratch-high.yaml --project /tmp/{} --noval --nosave'.format(
            yolo7_dir, yolo7_dir, target, target, batch_size, yolo7_dir, yolo7_dir, target)
        subprocess.call(train_cmd, shell=True)
    end_time = datetime.datetime.now()
    print(end_time - start_time)
