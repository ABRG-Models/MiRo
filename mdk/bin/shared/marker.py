import cv2
import copy
import numpy
import thread
import time
from datetime import datetime



kernel = numpy.ones((5,5),numpy.uint8)
#screen_lock = thread.allocate()
#DEPTH_WINSIZE = 480, 640



def drawTextInCorner(img, text):
    img = cv2.putText(img, text, (10,50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2, cv2.LINE_AA)
    return img


def handleKeyResponse(img, handStat, mask):
    key = chr(cv2.waitKey(10) & 0xFF)  # if 64 bit system, waitKey() gives result > 8 bits, ANDing with 11111111 removes extra ones

    if key == 'c':   handStat.calibrate(mask)
    elif handStat.isOnScreen(mask):
        if key == 'g': print handStat.getOpenFingers(mask)
        elif key == 'v':
            velocVec = handStat.getHandVelocityVec(sampleTimeMsec=150, sampIntervalMsec=10)
            print velocVec.toTuple() if velocVec != None else "no result"
        elif key == 'a':
            accVec = handStat.getHandAccelVec(sampleTimeMsec=200, sampIntervalMsec=10)
            print accVec.toTuple() if accVec != None else "no result"

def b_LargestContour(contours):
    max_area=0
    ci = -1
    cnt = []
    for i in range(len(contours)):
        cnt=contours[i]
        if not cv2.isContourConvex(cnt):
            area = cv2.contourArea(cnt)
            if(area>max_area):
                max_area=area
                ci=i
    
    if ci > -1:
        cnt=contours[ci]
    
    return cnt
 
def markerDetection(img, countourM, orgImg, showImg=False):
    centr = []
    biggestCnt = b_LargestContour(countourM)
    if biggestCnt.any():
        hull = cv2.convexHull(biggestCnt)
        moments = cv2.moments(biggestCnt)
        if moments['m00']!=0:
                    cx = int(moments['m10']/moments['m00']) # cx = M10/M00
                    cy = int(moments['m01']/moments['m00']) # cy = M01/M00
                  
        centr=(cx,cy)  
        if showImg:     
            #orgImg = cv2.circle(orgImg,centr,5,[0,0,255],2)          
            #orgImg = cv2.drawContours(orgImg,[biggestCnt],0,(0,255,0),2)
            #orgImg = cv2.drawContours(orgImg,[hull],0,(0,0,255),2)
            pass
    return orgImg, biggestCnt, centr

def markerRadius(img, cnt, centr, orgImg, showImg=False):
    cnt = cv2.approxPolyDP(cnt,0.01*cv2.arcLength(cnt,True),True)
    (x,y),radius = cv2.minEnclosingCircle(cnt)
    center = (int(x),int(y))
    radius = int(radius)
    if showImg and radius >= 1.0:
        orgImg = drawTextInCorner(orgImg, 'Circle Radius: %s' % radius)
        orgImg = cv2.circle(orgImg,center,2,(0,255,0),2)
        orgImg = cv2.circle(orgImg,center,radius,(0,255,0),2)
    return orgImg, radius, center

def get_mask( image, color ):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    s_min = 180
    s_max = 255
    h_wid = 20
    v_min = 80
    v_max = 220

    if color == 'blue':
        blue = 120
        h_wid = 30
        mask = cv2.inRange(hsv, (blue-h_wid, s_min, v_min), (blue+h_wid, s_max, v_max))
    elif color == 'red':
        mask1 = cv2.inRange(hsv, (0, s_min, v_min), (h_wid, s_max, v_max))
        mask2 = cv2.inRange(hsv, (180-h_wid, s_min, v_min), (180, s_max, v_max))
        mask = cv2.bitwise_or(mask1, mask2)

    else:
        mask = None
        
    return mask

def image_handler_function(video, showImg, mask):  
        
    # For laptop webcam
    #blue_mask = cv2.inRange(hsv, (109, 70 , 70, 0), (165, 255, 200, 0))
    # For logitech webcam without glass-screen
    #For logitech camera with glass screen
    #blue_mask = cv2.inRange(hsv, (110, 70 , 100, 0), (170, 255, 200, 0))
    #green_mask = cv2.inRange(hsv, (46, 70 , 30, 0), (100, 255, 200, 0))
    threshImg = cv2.erode(mask,kernel,iterations = 2)
    threshImg = cv2.dilate(threshImg,kernel,iterations = 2)
    #gray = cv2.cvtColor(video,cv2.COLOR_BGR2GRAY)
    
    image, contourMap, hierarchy = cv2.findContours(threshImg, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    img = cv2.cvtColor(threshImg, cv2.COLOR_GRAY2RGB)
    orgImg = copy.copy(video)
    radius = 0.0
    centre = None
          
    if len(contourMap) > 0: 
        orgImg, blueContour, blueCentre = markerDetection(img, contourMap, video, showImg)
        
        if len(blueContour) >0:
            orgImg, radius, centre = markerRadius(img, blueContour, blueCentre, video, showImg)
            if radius < 1.0:
                radius = 0.0
                centre = None
    returnImage = []
    if showImg:
        if radius > 0.0:
           returnImage = orgImg
        else:
           returnImage = video
    return radius, centre, returnImage
                
        

# def main():
#     cv2.namedWindow('image', cv2.WINDOW_AUTOSIZE)
#     #cam = cv2.VideoCapture(0) 
#     while True:
#         #ret_val, img = cam.read()
#         #img = cv2.flip(img, 1)
#         img = cv2.imread('no_ball.jpg', 1)
#         radius, centre, showImg = image_handler_function(img, True)
#         if showImg.any():
#             cv2.imshow('image', showImg)
#         if cv2.waitKey(1) == 27: 
#             break  # esc to quit
    
#     cv2.destroyAllWindows()
#     fp.close()

# if __name__ == '__main__':
#     main()
