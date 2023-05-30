from functools import lru_cache

from imgaug import augmenters as iaa
import numpy as np
from PIL import Image
from pybsc.image_utils import mask_to_bbox
from pybsc.image_utils import rescale
from pybsc.image_utils import resize_keeping_aspect_ratio_wrt_longside
from pybsc.image_utils import rotate
from pybsc.image_utils import squared_padding_image


@lru_cache(maxsize=None)
def cached_imread(img_path, image_width=300):
    pil_img = Image.open(img_path)
    img = np.array(pil_img)
    try:
        img, mask = img[..., :3], img[..., 3]
    except IndexError:
        mask = 255 * np.ones(
            (pil_img.height, pil_img.width),
            dtype=np.uint8)
    pil_mask = Image.fromarray(mask)
    pil_mask = resize_keeping_aspect_ratio_wrt_longside(
        pil_mask, image_width,
        interpolation='nearest')
    pil_img = Image.fromarray(img)
    pil_img = resize_keeping_aspect_ratio_wrt_longside(
        pil_img, image_width,
        interpolation='bilinear')
    return pil_img, pil_mask


def create_instance_image(instance_id, size):
    w, h = size
    return Image.fromarray(instance_id * np.ones((h, w), dtype=np.int32))


def random_rotate(pil_img, mask=None, angle=360):
    degrees = np.random.uniform(low=-angle, high=angle)
    return rotate(pil_img, mask, degrees)


def random_rescale(pil_img, mask=None, min_scale=0.2, max_scale=0.5):
    scale = np.random.uniform(min_scale, max_scale)
    return rescale(pil_img, mask, scale)


def random_crop_with_size(pil_img, image_width=300):
    w, h = pil_img.size
    if w < image_width or h < image_width:
        cv_img = squared_padding_image(np.array(pil_img), image_width)
        pil_img = Image.fromarray(cv_img)
        w, h = pil_img.size
    if w - image_width > 0:
        x = np.random.randint(0, w - image_width)
    else:
        x = 0
    if h - image_width > 0:
        y = np.random.randint(0, h - image_width)
    else:
        y = 0
    crop_img = pil_img.crop((x, y, x + image_width, y + image_width))
    return crop_img


def random_binpacking(pil_bg_img, img_paths, low=20, high=30, p=None,
                      targets=None,
                      image_width=300,
                      max_angle=360,
                      min_scale=0.2,
                      max_scale=0.5):
    pil_bg_img = pil_bg_img.copy()
    size = np.random.randint(low, high)
    names = []
    rectangles = []
    pil_imgs = []
    while len(rectangles) < size:
        path = np.random.choice(img_paths, p=p)
        pil_img, pil_mask = cached_imread(path, image_width=image_width)
        name = path.parent.name.lower()
        if targets is not None:
            if name not in targets:
                name = 'others'
        if pil_img.size[0] == 0 or pil_img.size[1] == 0:
            print('size 0 after augmentations. retry')
            continue
        pil_img, pil_mask = random_rescale(pil_img, mask=pil_mask,
                                           min_scale=min_scale,
                                           max_scale=max_scale)
        if pil_img.size[0] == 0 or pil_img.size[1] == 0:
            print('size 0 after rescale. retry')
            continue

        if np.random.uniform(0, 1.0) > 0.5:
            aug = iaa.PerspectiveTransform(scale=(0.1, 0.10), keep_size=False)
            _aug = aug._to_deterministic()
            pil_img = _aug.augment_image(np.array(pil_img, dtype=np.uint8))
            pil_img = Image.fromarray(pil_img)
            pil_mask = _aug.augment_image(np.array(pil_mask, dtype=np.uint8))
            pil_mask = Image.fromarray(pil_mask)

        if np.random.uniform(0, 1.0) > 0.5:
            aug = iaa.MultiplyAndAddToBrightness(mul=(0.1, 2.0), add=(-30, 30))
            pil_img = aug.augment_image(np.array(pil_img, dtype=np.uint8))
            pil_img = Image.fromarray(pil_img)

            aug = iaa.AddToSaturation()
            pil_img = aug.augment_image(np.array(pil_img, dtype=np.uint8))
            pil_img = Image.fromarray(pil_img)

        pil_img, pil_mask = random_rotate(pil_img, mask=pil_mask,
                                          angle=max_angle)

        if np.random.uniform(0, 1.0) > 0.5:
            # aug = iaa.PiecewiseAffine(scale=(0.01, 0.05))
            aug = iaa.ElasticTransformation()
            _aug = aug._to_deterministic()
            pil_img = _aug.augment_image(np.array(pil_img, dtype=np.uint8))
            pil_img = Image.fromarray(pil_img)
            pil_mask = _aug.augment_image(np.array(pil_mask, dtype=np.uint8))
            pil_mask = Image.fromarray(pil_mask)

        if np.random.uniform(0, 1.0) > 0.5:
            aug = iaa.Cutout(nb_iterations=(1, 5), size=0.2, squared=False,
                             fill_mode="constant", cval=(0, 255),
                             fill_per_channel=0.5)
            _aug = aug._to_deterministic()
            pil_img = _aug.augment_image(np.array(pil_img, dtype=np.uint8))
            pil_img = Image.fromarray(pil_img)

        if pil_img.size[0] == 0 or pil_img.size[1] == 0:
            print('size 0 after rotate. retry.')
            continue
        try:
            y1, x1, y2, x2 = mask_to_bbox(pil_mask)
        except ValueError as e:
            print('error on mask_to_bbox')
            print(str(e))
            continue
        h = y2 - y1
        w = x2 - x1
        pil_imgs.append((pil_img, pil_mask))
        rectangles.append((w, h))
        names.append(name)
    from rectpack import newPacker
    packer = newPacker()
    for rid, rect in enumerate(rectangles):
        packer.add_rect(rect[0], rect[1], rid=rid)
    packer.add_bin(*pil_bg_img.size)
    packer.pack()
    bboxes = []
    new_names = []
    instance_mask = Image.fromarray(
        np.zeros((pil_bg_img.size[1], pil_bg_img.size[0]), dtype=np.int32))
    instance_id = 1
    for r in packer.rect_list():
        if np.random.uniform(0, 1.0) > 0.8:
            continue
        index = r[5]
        x, y, w, h = r[1], r[2], r[3], r[4]
        pil_img, pil_mask = pil_imgs[index]
        y1, x1, y2, x2 = mask_to_bbox(pil_mask)
        if (x2 - x1) == h:
            pil_img = pil_img.rotate(90, resample=Image.BILINEAR, expand=True)
            pil_mask = pil_mask.rotate(90, resample=Image.NEAREST, expand=True)
        y1, x1, y2, x2 = mask_to_bbox(pil_mask)
        if names[index] in ['pocari', 'irohasu']:
            pil_mask = Image.fromarray(
                np.array(np.clip(np.array(pil_mask) * 0.5, 0, 255), dtype=np.uint8))

        pil_bg_img.paste(pil_img, (x - x1, y - y1), pil_mask)
        pil_mask = Image.fromarray(255 * np.array(np.array(pil_mask) > 0, dtype=np.uint8))
        instance_mask.paste(
            create_instance_image(instance_id, pil_img.size),
            (x - x1, y - y1), pil_mask)
        instance_id += 1
        bboxes.append([y, x, y + h, x + w])
        new_names.append(names[index])
    return pil_bg_img, bboxes, new_names, instance_mask
