#! /usr/bin/env python
import argparse
import json
import numpy as np

parser = argparse.ArgumentParser(description='Zip Detections Labels')
parser.add_argument('--start', dest='start', required=True, type=int)
parser.add_argument('--end', dest='end', required=True, type=int)
parser.add_argument('--points', dest='points', required=True, type=str)
parser.add_argument('--labels_file', dest='labelsFile',
                    required=True, type=str)
parser.add_argument('labels', nargs="*", type=str)
args = parser.parse_args()

with open(args.labelsFile, 'r') as f:
    labels = json.load(f)
labels = sorted(labels["value0"], key=lambda x: x["frame"])


is_label = {}
for label in args.labels:
    is_label[label] = []

for label in args.labels:
    for elem in labels:
        for currLabel in elem["labels"]:
            if currLabel["label"] == label:
                is_label[label].append(currLabel["value:"])

    is_label[label] = np.array(is_label[label])
    is_label[label] = is_label[label][args.start:args.end + 1]


pos = np.genfromtxt(args.points, delimiter=',')
data = pos
header = "x,y"
for label in args.labels:
    if label in is_label:
        min = np.minimum(len(data), len(is_label[label]))
        data = np.c_[data[:min], is_label[label][:min]]
        header = header + "," + label

np.savetxt("zipped.csv", data, delimiter=',', fmt="%i", header=header)
