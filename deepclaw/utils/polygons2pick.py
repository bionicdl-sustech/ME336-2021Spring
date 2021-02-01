# MIT License.
# Copyright (c) 2020 by BioicDL. All rights reserved.
# Created by LiuXb on 2020/12/21
# -*- coding:utf-8 -*-

"""
@Modified: 
@Description: transfer polygons label to mask
"""
import json
import os
import math
import numpy as np
import PIL.Image
import PIL.ImageDraw
import matplotlib.pyplot as plt
from sklearn.decomposition import PCA


# transfer polygons to mask image
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


class polygon2pick(object):
    def __init__(self, vis: bool = False):
        self.show_pca_flag = vis
        # new label data structure
        self.label_dict = {"version": [], "shapes": [], 'imagePath': [], "imageHeight": [], "imageWidth": []}
        # position is [x, y]; angle is the angle between the first principal component and x axis (width) ,(-pi/2, pi/2]
        # Note: the origin in image coordinate is top left, and in matplotlib plt is bottom left
        self.shape_dict = {"label": [], "position": [], "angle": []}

    def load_file(self, file_path: str):
        # json file
        with open(file_path) as f:
            data = json.load(f)

        version = data.get("version")
        img_Path = data.get("imagePath")
        img_width = data.get("imageWidth")
        img_height = data.get("imageHeight")
        # transfer label file to picking format
        self.label_dict["version"] = version
        self.label_dict["imagePath"] = img_Path
        self.label_dict["imageHeight"] = img_height
        self.label_dict["imageWidth"] = img_width

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
                theta = math.pi/2
            else:
                theta = math.atan(best_axis[1]/best_axis[0])  # (-pi/2, pi/2)
            # print('theta: ', theta*180/3.14)

            self.shape_dict["label"] = temp_label
            self.shape_dict["position"] = pca.mean_.tolist()  # col, row
            self.shape_dict["angle"] = theta
            shapes_temp.append(self.shape_dict.copy())

            # show pca axis
            if self.show_pca_flag:
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
        self.label_dict["shapes"] = shapes_temp
        return self.label_dict

    def save_file(self, file_path = None):
        if os.path.exists(file_path):
            print("File Already Exists! Check the file name!")
        else:
            with open(file_path, 'w') as f:
                json.dump(self.label_dict, f, indent=4)
        return self.label_dict


if __name__ == "__main__":
    """
    polygon format:
      "shapes": [
    {
      "label": "plastic",
      "points": [
        [
          391.358024691358,
          199.01234567901236
        ],
        [
          388.88888888888886,
          228.64197530864197
        ],
        [
          393.82716049382714,
          233.58024691358025
        ],
        [
          444.4444444444444,
          237.28395061728395
        ],
        [
          448.1481481481481,
          218.76543209876542
        ],
        [
          445.679012345679,
          205.18518518518516
        ]
      ],
      "group_id": null,
      "shape_type": "polygon",
      "flags": {}
    }
    ]
    picking format:
    "shapes": [
    {
        "label": "plastic",
        "position": [
            417.6426694692591,
            218.43194955333684
        ],
        "angle": 0.11164777833417805
    }
    ]
    """
    test = polygon2pick(True)
    test.load_file('/home/bionicdl-saber/Documents/data/ICRA2020/training_data/200918_14/labels_pick/00002200.json')
    test.save_file('./xx.json')
