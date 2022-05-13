import cv2
import numpy as np

def edge_detect(image):


    # load the image, convert it to grayscale, and blur it slightly
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    # show the original and blurred images
    cv2.imshow("Original", image)
    cv2.imshow("Blurred", blurred)

    # compute a "wide", "mid-range", and "tight" threshold for the edges
    # using the Canny edge detector
    wide = cv2.Canny(blurred, 10, 200, L2gradient=True)
    mid = cv2.Canny(blurred, 30, 150, L2gradient=True)
    tight = cv2.Canny(blurred, 240, 250, L2gradient=True)

    # show the output Canny edge maps
    #cv2.imshow("Wide Edge Map", wide)
    #cv2.imshow("Mid Edge Map", mid)
    #cv2.imshow("Tight Edge Map", tight)
    #cv2.waitKey(0)
    return tight

def find_side(image):
    y,x = image.shape
    compare_x = x
    for i in range(0,y):
        for j in range(0,x):
            if test[i][j] == 255:
                if j < compare_x:
                    compare_x = j
                    break
    compare_y = 0
    for i in range(0,y):
        for j in range(0,x):
            if test[i][j] == 255:
                if j > compare_y:
                    compare_y = j
    compare_y = x - compare_y
    diff = compare_x - compare_y
    if diff < 0:
        direction = 'R'
    else:
        direction = 'L'
    return direction

image = cv2.imread('ball.jpg')
test = edge_detect(image)
print(test.shape)
test2 = find_side(test)
print(test2)
cv2.imshow("Test", test)
cv2.waitKey(0)
