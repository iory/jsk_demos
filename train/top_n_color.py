from pathlib import Path
import random

import numpy as np
from pybsc import load_json
from pybsc import save_json
import scipy
import scipy.cluster

from aug import cached_imread


def pillow_image_to_simple_bitmap(pillow_image, mask=None):
    if mask is not None:
        bitmap = np.array(pillow_image, dtype=np.int16)[np.array(mask) > 150]
    else:
        bitmap = np.array(pillow_image, dtype=np.int16)
        shape = bitmap.shape
        bitmap = bitmap.reshape(scipy.product(shape[:2]), shape[2])
    bitmap = bitmap.astype(np.float)
    return bitmap


def top_n_colors(pillow_image, top_n, num_of_clusters, mask=None):
    clustering = scipy.cluster.vq.kmeans
    bitmap = pillow_image_to_simple_bitmap(pillow_image, mask)
    clusters, _ = clustering(bitmap, num_of_clusters)
    quntized, _ = scipy.cluster.vq.vq(bitmap, clusters)
    histgrams, _ = scipy.histogram(quntized, len(clusters))
    order = np.argsort(histgrams)[::-1][:top_n]
    for idx in range(min(top_n, len(order))):
        rgb = clusters.astype(int)[order[idx]].tolist()
        yield rgb


def take_random_n_color(path, n=10):
    path = Path(path)
    if path.with_suffix('.json').exists():
        colors = load_json(path.with_suffix('.json'))
    else:
        pil_img, pil_mask = cached_imread(path)
        c = n + 1
        colors = list(top_n_colors(pil_img, top_n=n, num_of_clusters=c, mask=pil_mask))
        save_json(colors, path.with_suffix('.json'))
    color = random.choice(colors)
    return color


if __name__ == '__main__':
    from PIL import Image
    IMG_SIZE = 64
    MARGIN = 0
    width = IMG_SIZE * 3 + MARGIN * 2
    height = IMG_SIZE + MARGIN * 2

    from eos import make_fancy_output_dir
    from eos import makedirs
    output_path = Path(make_fancy_output_dir('./results', no_save=True))
    for path in img_paths:
        tiled_color_img = Image.new(
            mode='RGB', size=(width, height), color='#333333')
        pil_img, pil_mask = cached_imread(path)
        n = 3
        c = n + 1
        colors = list(top_n_colors(pil_img, top_n=n, num_of_clusters=c, mask=pil_mask))

        for i, rgb_arr in enumerate(colors):
            color_hex_str = '#%02x%02x%02x' % tuple(rgb_arr)
            color_img = Image.new(
                mode='RGB', size=(IMG_SIZE, IMG_SIZE),
                color=color_hex_str)
            tiled_color_img.paste(
                im=color_img,
                box=(MARGIN + IMG_SIZE * i, MARGIN))
        makedirs(output_path / path.parent.name)
        tiled_color_img.save(output_path / path.parent.name / path.name)
