# import the necessary packages
from scipy.spatial import distance as dist
import numpy as np
import argparse
import cv2
import imutils

# load the image
image = cv2.imread("miro.png")
output = image.copy()

output = cv2.medianBlur(output,5)
imgHSV= cv2.cvtColor(output,cv2.COLOR_BGR2HSV)


imgGray= cv2.cvtColor(output,cv2.COLOR_BGR2GRAY)
imgGray = cv2.GaussianBlur(imgGray, (5,5), 0)
thresh = cv2.threshold(imgGray, 60, 255, cv2.THRESH_BINARY)[1]

# find contours in the thresholded image
cntss = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,
    cv2.CHAIN_APPROX_SIMPLE)
cntss = imutils.grab_contours(cntss)

# thresh = cv2.threshold(imgGray, 45, 255, cv2.THRESH_BINARY)[1]
# thresh = cv2.erode(thresh, None, iterations=2)
# thresh = cv2.dilate(thresh, None, iterations=2)
#
# cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,
#     cv2.CHAIN_APPROX_SIMPLE)




# show the output image
# cv2.imshow("Image", image)
# cv2.waitKey(0)
# cv2.imshow("detected object", imgHSV)
#     # cv2.imshow("fill gap", maskClose)
# cv2.waitKey(0)

# green color boundary (RGB)
# ([0, 127, 0], [180, 240, 180])

# white (probably some gray) color boundary (RGB)
# ([128, 128, 128], [255, 255, 255])

# White color boundary (HSV)
# ([0, 0, 195], [255, 60, 255])

# Orange color boundary (HSV)
# ([1, 190, 200], [25, 255, 255])


# define the list of boundaries
boundaries = [
    ([0, 0, 195], [255, 60, 255]),
    ([1, 190, 200], [25, 255, 255])
]

font = cv2.FONT_HERSHEY_SIMPLEX

count = 0

def find_center(cnts):
    for i in cnts:
    # compute the center of the contour
        M = cv2.moments(i)
        coX = int(M["m10"] / M["m00"])
        coY = int(M["m01"] / M["m00"])
        # draw the contour and center of the shape on the image
        cv2.circle(image, (coX, coY), 7, (255, 255, 255), -1)
        cv2.putText(image, "center", (coX - 20, coY - 20),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
    return coX, coY

find_center(cntss)
image_centerX, image_centerY = find_center(cntss)
print("X: ",image_centerX)
print("Y: ",image_centerY)

# loop over the boundaries
for (lower, upper) in boundaries:
    # create NumPy arrays from the boundaries
    lower = np.array(lower, dtype = "uint8")
    upper = np.array(upper, dtype = "uint8")

    # find the colors within the specified boundaries and apply
    # the mask
    mask = cv2.inRange(imgHSV, lower, upper)
    # output = cv2.bitwise_and(image, image, mask = mask)

    kernelOpen=np.ones((5,5))
    kernelClose=np.ones((20,20))
    maskOpen=cv2.morphologyEx(mask,cv2.MORPH_OPEN,kernelOpen)
    maskClose=cv2.morphologyEx(maskOpen,cv2.MORPH_CLOSE,kernelClose)

    maskFinal=maskClose.copy()
    im2, contours, hierarchy=cv2.findContours(maskFinal, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
    cnts = cv2.findContours(maskFinal, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    c = max(cnts, key=cv2.contourArea)

    # extLeft = tuple(c[c[:, :, 0].argmin()][0])
    # extRight = tuple(c[c[:, :, 0].argmax()][0])
    # extTop = tuple(c[c[:, :, 1].argmin()][0])
    # extBot = tuple(c[c[:, :, 1].argmax()][0])

    for i in range(len(contours)):
        if count == 0:
            text = "MiRO"
        else:
            text = "Football"

        cv2.drawContours(image, [c], -1, (0, 255, 255), 2)
        # cv2.circle(image, extLeft, 8, (0, 0, 255), -1)
        # cv2.circle(image, extRight, 8, (0, 255, 0), -1)
        # cv2.circle(image, extTop, 8, (255, 0, 0), -1)
        # cv2.circle(image, extBot, 8, (255, 255, 0), -1)

        x,y,w,h=cv2.boundingRect(contours[i])
        # cv2.rectangle(image,(x,y),(x+w,y+h),(0,0,255), 2)
        cv2.putText(image, text,(x,y+h),font,1.0,(0,255,255), True)
    count += 1
    # loop over the contours
    find_center(cnts)
    object_centerX, object_centerY = find_center(cnts)
    print("oX: ",object_centerX)
    print("oY: ",object_centerY)
    cv2.line(image, (int(image_centerX), int(image_centerY)), (int(object_centerX), int(object_centerY)),
            (255, 0, 0), 2)
    D = dist.euclidean((image_centerX, image_centerY), (object_centerX, object_centerY))
    # cv2.putText(image, "{:.1f}in".format(D), (int(object_centerX), int(object_centerY - 10)),
    #             cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 0, 0), 2)
    print("Distance: ", D)
    if object_centerX>image_centerX:
        print("right")
        print("direction vector: ",object_centerX-image_centerX)
        if object_centerY>image_centerY:
            print("down")
            print("direction vector: ",object_centerY-image_centerY)
        elif object_centerY<image_centerY:
            print("up")
            print("direction vector: ",image_centerY-object_centerY)
    elif object_centerX<image_centerX:
        print("left")
        print("direction vector: ",image_centerX-object_centerX)
        if object_centerY>image_centerY:
            print("down")
            print("direction vector: ",object_centerY-image_centerY)
        elif object_centerY<image_centerY:
            print("up")
            print("direction vector: ",image_centerY-object_centerY)
    elif object_centerX==image_centerX:
        if object_centerY>image_centerY:
            print("down")
            print("direction vector: ",object_centerY-image_centerY)
        elif object_centerY<image_centerY:
            print("up")
            print("direction vector: ",image_centerY-object_centerY)
        else:
            print("right at the front")

cv2.imshow("detected object", image)
cv2.waitKey(0)

