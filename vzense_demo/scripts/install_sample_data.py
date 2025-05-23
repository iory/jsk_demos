#!/usr/bin/env python

import argparse
import multiprocessing
import os.path as osp

import jsk_data


def main():
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        '--pkg-path', help='PAKCAGE_SOURCE_DIR in cmake'
    )
    parser.add_argument('-v', '--verbose', dest='quiet', action='store_false')
    args = parser.parse_args()
    quiet = args.quiet
    pkg_path = args.pkg_path

    def download_data(**kwargs):
        path = kwargs.pop('path')
        if pkg_path is not None:
            path = osp.join(pkg_path, path)
        kwargs['path'] = path
        kwargs['pkg_name'] = 'jsk_pcl_ros_utils'
        kwargs['quiet'] = quiet
        p = multiprocessing.Process(
            target=jsk_data.download_data,
            kwargs=kwargs)
        p.start()

    download_data(
        path='sample/data/2017-02-05-16-11-09_shelf_bin.bag',
        url='https://drive.google.com/uc?id=1LhuGYlPNXEJW-G3dOwxD60z3k9KvYRxg',
        md5='44427634f57ac76111edabd7b1f4e140',
    )


if __name__ == '__main__':
    main()
