import cv2
import numpy as np
# import sys
DEBUG = True
class DetectForeground(object):
    def __init__(self):
        pass
    def DiffGround(self,groundImg,currrentImg):
        groundImg_gray = cv2.cvtColor(groundImg,cv2.COLOR_BGR2GRAY)
        groundBlur = cv2.GaussianBlur(groundImg_gray,(5,5),1)
        groundBlur.dtype = 'int16'
        # cv2.imshow('img',groundBlur)
        # cv2.waitKey()
        currrentImg_gray = cv2.cvtColor(currrentImg,cv2.COLOR_BGR2GRAY)
        currrentImgBlur = cv2.GaussianBlur(currrentImg_gray,(5,5),1)
        currrentImgBlur.dtype = 'int16'
        # cv2.imshow('img',currrentImgBlur)
        # cv2.waitKey()
        dGrayBlur = abs(groundBlur-currrentImgBlur)
        dGrayBlur.dtype = 'uint8'
        dGrayMidBlur=cv2.medianBlur(dGrayBlur,5)
        # cv2.imshow('img',dGrayMidBlur)
        # cv2.waitKey()

        ret,thresh=cv2.threshold(dGrayMidBlur,10,255,cv2.THRESH_BINARY)
        # cv2.imshow('img',thresh)
        # cv2.waitKey()
        result = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        if (len(result)==2):
            contours = result[0]
            hierarchy = result[1]
        elif(len(result)==3):
            contours = result[1]
            hierarchy = result[2]
        rect = []
        for i in range(len(contours)):
            area = cv2.contourArea(contours[i])
            if area < 40*40 :
                continue
            else:
                temp = cv2.boundingRect(contours[i])
                rect.append(temp)
        # x,y,w,h
        return rect

    def MOG2(self,groundImg,currrentImg,history = 1,kernel = 16):
        #TODO: complete this function for avi and a set of picture
        a = cv2.imread('image_C_0002.jpg',1)
        #histroy: use how many image to build the model
        #kernel: use how many gauss function to build the model
        fgbg = cv2.createBackgroundSubtractorMOG2(history,kernel,False)
        # 1 means add this picture to model
        fgmask = fgbg.apply(a)
        b = cv2.imread('image_C_0003.jpg',1)
        fgmask = fgbg.apply(b,1)

        b = cv2.imread('image_C_0001.jpg',1)
        # 0 means don't add this picture to model
        fgmask = fgbg.apply(b,fgmask,0)

    def ColorFilter(self,currrentImg,lower = np.array([10, 80, 0]),upper = np.array([80, 180, 80])):
        mask = cv2.inRange(currrentImg, lower, upper)
        mask = cv2.bitwise_not(mask)

        if(DEBUG):
            cv2.imshow('mask',mask)
            cv2.waitKey()

        contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

        # # contour: if hierarchy[0][i][3] == -1, it means there are contours inside
        # # len(contours[i] is the num of the contour
        # if (len(contours[i]) < self.min or len(contours[i]) > self.max or cv2.contourArea(contours[i]) < self.min_area): #or hierarchy[0][i][3] == -1
        #     continue
        rect = []
        for i in range(len(contours)):
            area = cv2.contourArea(contours[i])
            if area < 30*30 or area > 200*200:
                continue
            else:
                temp = cv2.boundingRect(contours[i])
                rect.append(temp)
        return rect

if __name__ == '__main__':
    a = cv2.imread('../Data/BoundingBox1.png')
    rect = DetectForeground().ColorFilter(a)
    print(rect)
    showed_image = a.copy()
    for i in range(len(rect)):
        cv2.rectangle(showed_image,(rect[i][0],rect[i][1]),(rect[i][0]+rect[i][2],rect[i][1]+rect[i][3]),(0,255,0),2)
    cv2.imshow('contours',showed_image)
    cv2.waitKey()
    cv2.destroyAllWindows()
