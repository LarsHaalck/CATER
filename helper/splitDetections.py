#! /usr/bin/env python
import argparse
import numpy as np

parser = argparse.ArgumentParser(description='Split Detections w.r.t. to numbers')
parser.add_argument('--detections', dest='detectFile',
                    required=True, type=str)
parser.add_argument('sizes', nargs="*", type=int)
args = parser.parse_args()

sizes = np.pad(np.cumsum(args.sizes), (1, 0))

data = np.genfromtxt(args.detectFile, dtype=int, delimiter=',')
for i in range(len(sizes) - 1):
    filename = "{:02d}.csv".format(i)
    # print(filename)
    np.savetxt(filename, data[sizes[i]:sizes[i+1]], delimiter=',', fmt="%i")


