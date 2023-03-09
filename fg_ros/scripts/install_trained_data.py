#!/usr/bin/env python

from __future__ import print_function

import argparse
import multiprocessing

import jsk_data


def download_data(*args, **kwargs):
    p = multiprocessing.Process(
            target=jsk_data.download_data,
            args=args,
            kwargs=kwargs)
    p.start()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-v', '--verbose', dest='quiet', action='store_false')
    args = parser.parse_args()
    args.quiet

    PKG = 'fg_ros'

    download_data(
        pkg_name=PKG,
        path='trained_data/yolo7/2023-03-09.pt',
        url='https://drive.google.com/uc?id=1uS_TGHHKvSo1Il8V7uukCBaJ5ngfGQw0',
        md5='3d95fec634d99a7cbb861d822135e279',
    )


if __name__ == '__main__':
    main()
