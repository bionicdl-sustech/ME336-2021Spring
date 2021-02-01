import cv2
import torch
from numpy import random
import os

from models.experimental import attempt_load
from utils.general import (
    check_img_size, non_max_suppression, apply_classifier, scale_coords,
    xyxy2xywh, plot_one_box, strip_optimizer, set_logging)
from utils.torch_utils import select_device, load_classifier, time_synchronized
import yaml
from utils.datasets import letterbox
import numpy as np


class Yolo5(object):
    @torch.no_grad()
    def __init__(self, cfg_path):
        __cfg = yaml.load(open(cfg_path, 'r'), Loader=yaml.FullLoader)
        self.weights = __cfg['weights']
        self.img_size = __cfg['img_size']
        self.conf_thres = __cfg['conf_thres']
        self.iou_thres = __cfg['iou_thres']
        self.device = __cfg['device']
        self.view_img = __cfg['view_img']
        self.print_result = __cfg['print_result']
        self.classes = __cfg['classes']
        self.agnostic_nms = __cfg['agnostic_nms']
        self.augment = __cfg['augment']
        self.update = __cfg['update']
        # Initialize
        set_logging()
        self.device = select_device(self.device)
        self.half = self.device.type != 'cpu'  # half precision only supported on CUDA
        # Load model
        self.model = attempt_load(self.weights, map_location=self.device)  # load FP32 model

    @torch.no_grad()
    def detect(self, ori_img):
        view_img, print_result, imgsz = self.view_img, self.print_result, self.img_size
        imgsz = check_img_size(imgsz, s=self.model.stride.max())  # check img_size
        if self.half:
            self.model.half()  # to FP16
        # Get names and colors
        names = self.model.module.names if hasattr(self.model, 'module') else self.model.names
        colors = [[random.randint(0, 255) for _ in range(3)] for _ in range(len(names))]
        # Run inference
        img = torch.zeros((1, 3, imgsz, imgsz), device=self.device)  # init img
        _ = self.model(img.half() if self.half else img) if self.device.type != 'cpu' else None  # run once

        # load image
        im0 = ori_img.copy()
        img = letterbox(im0, imgsz)[0]
        # Convert
        img = img[:, :, ::-1].transpose(2, 0, 1)  # BGR to RGB, to 3x416x416
        img = np.ascontiguousarray(img)
        img = torch.from_numpy(img).to(self.device)
        img = img.half() if self.half else img.float()  # uint8 to fp16/32
        img /= 255.0  # 0 - 255 to 0.0 - 1.0
        if img.ndimension() == 3:
            img = img.unsqueeze(0)

        # Inference
        t1 = time_synchronized()
        # pred is  ux,uy,vx,vy, confidence, probability of each class
        pred = self.model(img, augment=self.augment)[0]
        # Apply NMS
        pred = non_max_suppression(pred, self.conf_thres, self.iou_thres, classes=self.classes, agnostic=self.agnostic_nms)
        t2 = time_synchronized()
        # Process detections
        det = []
        for i, det in enumerate(pred):  # detections per image
            s = '%gx%g ' % img.shape[2:]  # print string
            gn = torch.tensor(im0.shape)[[1, 0, 1, 0]]  # normalization gain whwh
            if det is not None and len(det):
                # Rescale boxes from img_size to im0 size
                det[:, :4] = scale_coords(img.shape[2:], det[:, :4], im0.shape).round()
                if print_result:
                    print(det)
                # Print results
                for c in det[:, -1].unique():
                    n = (det[:, -1] == c).sum()  # detections per class
                    s += '%g %ss, ' % (n, names[int(c)])  # add to string

                # show results
                for *xyxy, conf, cls in reversed(det):
                    xywh = (xyxy2xywh(torch.tensor(xyxy).view(1, 4)) / gn).view(-1).tolist()  # normalized xywh
                    # print(('%g ' * 5 + '\n') % (cls, *xywh))
                    label = '%s %.2f' % (names[int(cls)], conf)
                    plot_one_box(xyxy, im0, label=label, color=colors[int(cls)], line_thickness=3)
                    # print(xyxy, np.array(torch.Tensor.cpu(cls).data), np.array(torch.Tensor.cpu(conf)))

            # Print time (inference + NMS)
            print('%sDone. (%.3fs)' % (s, t2 - t1))
            # Stream results
            if view_img:
                cv2.imshow('result', im0)
                cv2.waitKey()
        # ux,uy,vx,vy, confidence, class
        # col_min, row_min, col_max, row_max, confidence, class
        if det is None:
            return None
        else:
            return np.array(torch.Tensor.cpu(det))


if __name__ == '__main__':
    _root_path = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    _root_path = os.path.dirname(_root_path)
    _root_path = os.path.dirname(_root_path)
    os.chdir(_root_path)
    with torch.no_grad():
        kk = Yolo5('./configs/ICRA2020-ur5-azure-rg6/detection_cfg.yaml')
        test_img = cv2.imread('./deepclaw/modules/end2end/yolov5/inference/images/bus.jpg')
        m = kk.detect(test_img)
        for i in range(len(m)):
            uv = np.array(m[i][0:4], np.int16)
            cla = int(m[i][5])
            cfi = m[i][4]
            title = 'class: '+str(cla)+'  confidence: ' + str(cfi)
            cv2.imshow(title, test_img[uv[1]:uv[3], uv[0]:uv[2]])
            cv2.waitKey()
