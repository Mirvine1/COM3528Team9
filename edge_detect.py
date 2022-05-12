import cv2

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
    cv2.imshow("Wide Edge Map", wide)
    cv2.imshow("Mid Edge Map", mid)
    cv2.imshow("Tight Edge Map", tight)
    cv2.waitKey(0)
    return tight

image = cv2.imread('blob.png')
test = edge_detect(image)
cv2.imshow("Test", test)
cv2.waitKey(0)