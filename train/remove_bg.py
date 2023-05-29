from math import atan2
from math import cos
from math import pi
from math import sin
from math import sqrt

import cv2
import numpy as np
from pybsc.image_utils import rotate
from rembg import remove
from skimage import measure


def draw_axis(img, p_, q_, colour, scale):
    p = list(p_)
    q = list(q_)

    angle = atan2(p[1] - q[1], p[0] - q[0])  # angle in radians
    hypotenuse = sqrt((p[1] - q[1]) * (p[1] - q[1]) + (p[0] - q[0]) * (p[0] - q[0]))
    # Here we lengthen the arrow by a factor of scale
    q[0] = p[0] - scale * hypotenuse * cos(angle)
    q[1] = p[1] - scale * hypotenuse * sin(angle)
    cv2.line(img, (int(p[0]), int(p[1])), (int(q[0]), int(q[1])), colour, 1, cv2.LINE_AA)
    # create the arrow hooks
    p[0] = q[0] + 9 * cos(angle + pi / 4)
    p[1] = q[1] + 9 * sin(angle + pi / 4)
    cv2.line(img, (int(p[0]), int(p[1])), (int(q[0]), int(q[1])), colour, 1, cv2.LINE_AA)
    p[0] = q[0] + 9 * cos(angle - pi / 4)
    p[1] = q[1] + 9 * sin(angle - pi / 4)
    cv2.line(img, (int(p[0]), int(p[1])), (int(q[0]), int(q[1])), colour, 1, cv2.LINE_AA)


def get_orientation(pts, img):
    sz = len(pts)
    data_pts = np.empty((sz, 2), dtype=np.float64)
    for i in range(data_pts.shape[0]):
        data_pts[i, 0] = pts[i, 0, 0]
        data_pts[i, 1] = pts[i, 0, 1]
    # Perform PCA analysis
    mean = np.empty((0))
    mean, eigenvectors, eigenvalues = cv2.PCACompute2(data_pts, mean)
    # Store the center of the object
    cntr = (int(mean[0,0]), int(mean[0,1]))
    # cv2.circle(img, cntr, 3, (255, 0, 255), 2)
    p1 = (cntr[0] + 0.02 * eigenvectors[0,0] * eigenvalues[0,0], cntr[1] + 0.02 * eigenvectors[0,1] * eigenvalues[0,0])
    p2 = (cntr[0] - 0.02 * eigenvectors[1,0] * eigenvalues[1,0], cntr[1] - 0.02 * eigenvectors[1,1] * eigenvalues[1,0])
    draw_axis(img, cntr, p1, (0, 255, 0), 1)
    draw_axis(img, cntr, p2, (255, 255, 0), 5)
    angle = atan2(eigenvectors[0,1], eigenvectors[0,0])
    return angle


def remove_background(img, path_name=None,
                      return_info=False,
                      kernel_size=5,
                      iterations=4):
    img = remove(img)

    kernel = np.ones((kernel_size, kernel_size))
    mask = 255 * np.array(img[..., 3] > 0, dtype=np.uint8)
    img_dil = cv2.erode(mask, kernel, iterations=iterations)
    img_opening = cv2.dilate(img_dil, kernel, iterations=iterations)
    labels = measure.label(img_opening, background=0)
    mask_area = [np.sum(labels == i) for i in range(1, np.max(labels) + 1)]
    largest_mask_label = np.argmax(mask_area) + 1
    final_mask = (labels == largest_mask_label).astype(np.uint8) * 255
    y, x = np.where(final_mask > 0)

    x1 = np.min(x)
    x2 = np.max(x)
    y1 = np.min(y)
    y2 = np.max(y)
    img = img[y1:y2, x1:x2]

    mask_copy = ((img[..., 3] > 0).copy())
    mask_copy = 255 * np.array(mask_copy, dtype=np.uint8)

    contours, _ = cv2.findContours(
        mask_copy, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    areas = [cv2.contourArea(c) for i, c in enumerate(contours)]
    contour = contours[np.argmax(areas)]
    hoge_img = img.copy()[..., :3]

    rect = cv2.minAreaRect(contour)
    box = cv2.boxPoints(rect)
    box = np.int0(box)

    hoge_img = np.array(hoge_img, dtype=np.uint8)
    hoge_img = cv2.drawContours(
        hoge_img, contours,
        int(np.argmax(areas)), (0, 0, 255), 3)
    angle = get_orientation(contour, hoge_img)
    angle = get_orientation(box.reshape(-1, 1, 2), hoge_img)
    if path_name is not None:
        cv2.imwrite(str(path_name), hoge_img)
    angle = np.rad2deg(angle)
    img = rotate(img, angle=angle)
    if return_info:
        return img, (x1, y1, x2, y2), angle, mask
    return img


if __name__ == '__main__':
    import argparse
    from pathlib import Path

    from eos import make_fancy_output_dir
    from eos import makedirs

    parser = argparse.ArgumentParser(description='remove bg')
    parser.add_argument('targetpath', type=str)
    args = parser.parse_args()


    outpath = Path(make_fancy_output_dir('./rembg_img', no_save=True))
    paths = list(sorted(Path(args.targetpath).glob('*/*.jpg'))) \
        + list(sorted(Path(args.targetpath).glob('*/*.jpeg'))) \
        + list(sorted(Path(args.targetpath).glob('*/*.png')))
    for path in paths:
        print(path.name)
        try:
            makedirs(outpath / path.parent.name)
            out_img = remove_background(cv2.imread(str(path)))

            cv2.imwrite(str(outpath / path.parent.name / path.with_suffix('.png').name), out_img)
        except Exception as e:
            print(str(e))
