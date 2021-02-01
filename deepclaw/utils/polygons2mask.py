# MIT License.
# Copyright (c) 2020 by BioicDL. All rights reserved.
# Created by LiuXb on 2020/12/21
# -*- coding:utf-8 -*-

"""
@Modified: 
@Description: transfer polygons label to mask
"""

import io
import json
import os.path as osp
import math

import numpy as np
import PIL.Image
import PIL.ImageDraw
import matplotlib.pyplot as plt
from sklearn.decomposition import PCA


def shape_to_mask(
        img_shape, points, shape_type=None, line_width=10, point_size=5
):
    mask = np.zeros(img_shape[:2], dtype=np.uint8)
    mask = PIL.Image.fromarray(mask)
    draw = PIL.ImageDraw.Draw(mask)
    xy = [tuple(point) for point in points]
    if shape_type == "circle":
        assert len(xy) == 2, "Shape of shape_type=circle must have 2 points"
        (cx, cy), (px, py) = xy
        d = math.sqrt((cx - px) ** 2 + (cy - py) ** 2)
        draw.ellipse([cx - d, cy - d, cx + d, cy + d], outline=1, fill=1)
    elif shape_type == "rectangle":
        assert len(xy) == 2, "Shape of shape_type=rectangle must have 2 points"
        draw.rectangle(xy, outline=1, fill=1)
    elif shape_type == "line":
        assert len(xy) == 2, "Shape of shape_type=line must have 2 points"
        draw.line(xy=xy, fill=1, width=line_width)
    elif shape_type == "linestrip":
        draw.line(xy=xy, fill=1, width=line_width)
    elif shape_type == "point":
        assert len(xy) == 1, "Shape of shape_type=point must have 1 points"
        cx, cy = xy[0]
        r = point_size
        draw.ellipse([cx - r, cy - r, cx + r, cy + r], outline=1, fill=1)
    else:
        assert len(xy) > 2, "Polygon must have points more than 2"
        draw.polygon(xy=xy, outline=1, fill=1)
    mask = np.array(mask, dtype=bool)
    return mask


label_dict = {"version": [], "shapes": [], 'imagePath': [], "imageHeight": [], "imageWidth": []}
shape_dict = {"label": [], "position": [], "angle": []}


filename = './2007_003876.json'
with open(filename) as f:
    data = json.load(f)
version = data.get("version")
img_Path = data.get("imagePath")
img_width = data.get("imageWidth")
img_height = data.get("imageHeight")

# transfer label file to picking format
label_dict["version"] = version
label_dict["imagePath"] = img_Path
label_dict["imageHeight"] = img_height
label_dict["imageWidth"] = img_width

img_size = [img_width, img_height]
img_label_info = data.get("shapes")
shapes_temp = []
for i in range(len(img_label_info)):
    temp_label = img_label_info[i]["label"]
    temp_polygon = img_label_info[i]["points"]
    temp_mask = shape_to_mask(img_size, temp_polygon)  # binary image, object mask
    index_row, index_col = np.where(temp_mask == True)

    x_data = np.zeros([len(index_row), 2])  # row, col
    x_data[:, 1] = index_row
    x_data[:, 0] = index_col

    # grasping position and angle
    pca = PCA(n_components=2)
    pca.fit(x_data)

    best_axis = pca.components_[0]  # 长轴方向
    if best_axis[0] == 0:
        theta = math.pi
    else:
        theta = math.atan(best_axis[1]/best_axis[0])  # (-pi/2, pi/2)
    print('theta: ', theta*180/3.14)

    shape_dict["label"] = temp_label
    shape_dict["position"] = pca.mean_  # col, row
    shape_dict["angle"] = theta

    shapes_temp.append(shape_dict)

    # show pca axis
    show_pca_flag = False
    if show_pca_flag:
        print('pca.components_\n', pca.components_)
        print('pca.explained_variance_\n', pca.explained_variance_)
        print('pca.mean\n', pca.mean_)
        ax1 = plt.axes()
        plt.scatter(index_col, index_row, alpha=0.2)
        plt.ylim(img_height, 0)
        plt.xlim(0, img_width)
        for length, vector in zip(pca.explained_variance_, pca.components_):
            print('vector: ', vector)
            v = vector * 3 * np.sqrt(length)
            ax1.arrow(pca.mean_[0], pca.mean_[1], v[0], v[1], head_width=0.05, head_length=0.1, fc='k', ec='k')
            plt.scatter(pca.mean_[0], pca.mean_[1])

        plt.axis('equal')
        plt.show()

label_dict["shapes"] = shapes_temp
print(label_dict)



