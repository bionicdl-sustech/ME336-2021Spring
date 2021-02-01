import cv2
import numpy as np
from matplotlib import pyplot as plt
import time
import DetectForeground as df


area_threshold = 0
def success_label(img1,img2,compare_space=[630,830,50,100]):
    img1 = img1[compare_space[0]:compare_space[1],compare_space[2]:compare_space[3]]
    img2 = img2[compare_space[0]:compare_space[1],compare_space[2]:compare_space[3]]

    compare = df.Segment()
    rect = compare.DiffGround(img1,img2)
    if len(rect) > area_threshold :
        success_label = 1
        print('success_label:'+str(success_label))
        return success_label,img1,img2
    else :
        success_label = 0
        print('success_label:'+str(success_label))
        return success_label,img1,img2

if __name__=="__main__":
    # reference image
    a = cv2.imread('../Data/BoundingBox1.png')
    # pending iamge
    b = cv2.imread('../Data/BoundingBox2.png')
    # ROI
    ws = [630,830,50,100]
    sl = success_label(a,b,ws)
