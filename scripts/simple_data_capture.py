#!/usr/bin/env python
"""
simple_data_capture.py
Authored by Yoonyoung Cho @ 09.27.2018
Multi-Orientation data capture with bounds visualization.
WARNING : use for data collection on lightweight classification tasks only.
"""
import cv2
import numpy as np
from argparse import ArgumentParser
from collections import defaultdict
import os

def rint(x):
    return np.round(x).astype(np.int32)

def lsdir(x):
    return [name for name in os.listdir(x) if os.path.isfile(os.path.join(x,name))]

def main():
    # handle optional parameters
    parser = ArgumentParser()
    parser.add_argument('-s', '--src', default='/dev/video0', help='image stream source')
    parser.add_argument('-d', '--dir', default='/tmp/cap', help='root directory')
    parser.add_argument('-e', '--ext', default='png', help='image extension')
    parser.add_argument('-n', '--num', default=5, help='number of rotations')
    parser.add_argument('-c', '--cls', default='', help='list of classes')
    opt = parser.parse_args()

    # keep track of categorial counts
    cnt = defaultdict(lambda:0)

    # create dataset directory
    if not os.path.exists(opt.dir):
        os.makedirs(opt.dir)
    for cls in opt.cls.split(';'):
        if len(cls) <= 0: continue # invalid class label

        cls_path = os.path.join(opt.dir, cls)
        if not os.path.exists(os.path.join(opt.dir, cls)):
            os.makedirs(os.path.join(opt.dir, cls))

        # initialize with # of files in target directory
        cnt[cls] = len(lsdir(cls_path))
    print 'cnt', cnt

    cap = cv2.VideoCapture(opt.src)
    root = opt.dir

    s = 1.0 / np.sqrt(2)

    # initialization + cache data
    init = False
    n_i, n_j = 0,0
    cx, cy = 0,0
    r_ex, r_in = 0,0
    degs = np.linspace(0, 360, opt.num, endpoint=False)
    Ms = []

    while True:
        ret, frame = cap.read()
        if not ret:
            print 'frame capture unsuccessful'

        if not init:
            # figure out dimensions
            n_i, n_j = frame.shape[:2]
            cx, cy = (n_j/2.), (n_i/2.)
            r_ex = min(cx,cy)
            r_in = r_ex / np.sqrt(2)
            Ms = [cv2.getRotationMatrix2D((cx,cy),d,1) for d in degs]
            # set flag
            init = True

        # visualization ...
        viz = frame.copy()
        # interior circle
        cv2.circle(viz, tuple(rint([cx, cy])),
                radius=rint(r_in),
                color=(255,0,0),
                thickness=1)
        cv2.imshow('viz', viz)

        # interior rectangle - must fall within
        cv2.rectangle(viz,
                tuple(rint([cx-r_in,cy-r_in])),
                tuple(rint([cx+r_in,cy+r_in])),
                color=(0,0,255),
                thickness=1)

        # exterior circle
        cv2.circle(viz, tuple(rint([cx, cy])),
                radius=rint(r_ex),
                color=(255,0,0),
                thickness=1)
        cv2.imshow('viz', viz)

        k = cv2.waitKey(10)
        if k == 27: # esc
            break
        elif k == 32: # space
            print 'save!'
            rots = [cv2.warpAffine(frame, M, (n_j,n_i)) for M in Ms]
            for rot in rots:
                path = os.path.join(opt.dir, cls, '{}.{}'.format(cnt[cls], opt.ext))
                crop = rot[rint(cy-r_in):rint(cy+r_in),rint(cx-r_in):rint(cx+r_in)]
                cv2.imwrite(path,crop)
                cnt[cls] += 1

if __name__ == "__main__":
    main()
