import cv2
import numpy as np

def ExtractRedMask(img):
    img = np.asarray(img, np.uint8)
    
    # Convert to HSV Colormap
    img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # Extract locations with a green color
    mask1 = cv2.inRange(img_hsv, (0,50,20), (5,255,255))
    mask2 = cv2.inRange(img_hsv, (175,50,20), (180,255,255))
    maskColor = cv2.bitwise_or(mask1, mask2)

    # Get mask from selected pixels
    maskColor = maskColor > 0
    return maskColor

def FindRedFrame(img):
    # Filter noise in the image
    blurred_img = cv2.medianBlur(img, 3)

    # Extract binary mask of red pixels
    red_mask = ExtractRedMask(blurred_img)

    # Keep intact copy of image
    copy_img = img.copy()
    
    # Make red pixels white
    img[red_mask] = (255, 255, 255)

    # Make the other pixels black
    img[np.bitwise_not(red_mask)] = (0, 0, 0)

    # Convert to grayscale and binarize image
    img_grayscale = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ret, bin_img = cv2.threshold(img_grayscale, 100, 255, cv2.THRESH_BINARY)

    # Make an opening to remove noise and small red clusters of pixels
    kernel = np.ones((3,3), np.uint8)
    opening = cv2.morphologyEx(bin_img, cv2.MORPH_OPEN, kernel, iterations = 2)
    closing = cv2.morphologyEx(bin_img, cv2.MORPH_CLOSE, kernel, iterations = 2)

    # Find contours
    img_contours, contours, hierarchy = cv2.findContours(closing, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # Sort contours by size and extract the second longest contour
    contours = sorted(contours, key = cv2.contourArea, reverse = True)
    if countours is not None and len(contours) > 1:
        cont = contours[1]
        return cont
    return None

def DrawRedFrame(contourFrame, imgFrame):
    if contourFrame is None:
        cv2.namedWindow("Red Frame", cv2.WINDOW_NORMAL)  
        cv2.imshow("Red Frame", imgFrame)
        cv2.waitKey(3)
        return
    
    # Get bounding rotated rectangle
    #rect = cv2.minAreaRect(contourFrame[1])
    rect = cv2.minAreaRect(contourFrame)
    box = cv2.boxPoints(rect)
    box = np.int0(box)

    # Compare areas between bounding figures
    #trapezoid_area = cv2.contourArea(contourFrame[1])
    trapezoid_area = cv2.contourArea(contourFrame)
    rectangle_area = cv2.contourArea(box)
    #print("Area ratio: ", trapezoid_area/rectangle_area)

    # Find centroid of rectangle
    center_x = rect[0][0]
    center_y = rect[0][1]
    print(center_x, center_y)
    cv2.circle(imgFrame, (round(center_x), round(center_y)), 6, (0,0,255), -1)
    
    # Draw inner contour and its bounding rectangle
    #imgFrame = cv2.drawContours(imgFrame, [contours[1]], -1, (0,255,0), 3)
    imgFrame = cv2.drawContours(imgFrame, [contourFrame], -1, (0,255,0), 3)
    imgFrame = cv2.drawContours(imgFrame, [box], -1, (255,0,0), 3)
    cv2.namedWindow("Red Frame", cv2.WINDOW_NORMAL)  
    cv2.imshow("Red Frame", imgFrame)
    cv2.waitKey(3)
    #cv2.resizeWindow("Red Frame", (400,300))

if __name__ == "__main__":
    img = cv2.imread("RedFrameSample6.jpg", 1)
    
    # Get bounding polygon of inner contour
    #peri = cv2.arcLength(contours[1], True)
    #approx = cv2.approxPolyDP(contours[1], 0.001 * peri, True)
    
    # Get bounding rotated rectangle
    rect = cv2.minAreaRect(contours[1])
    box = cv2.boxPoints(rect)
    box = np.int0(box)

    # Compare areas between bounding figures
    trapezoid_area = cv2.contourArea(contours[1])
    rectangle_area = cv2.contourArea(box)
    print("Area ratio: ", trapezoid_area/rectangle_area)

    # Find centroid of rectangle
    center_x = rect[0][0]
    center_y = rect[0][1]
    print(center_x, center_y)
    cv2.circle(copy_img, (round(center_x), round(center_y)), 6, (0,0,255), -1)
    
    # Draw inner contour and its bounding rectangle
    copy_img = cv2.drawContours(copy_img, [contours[1]], -1, (0,255,0), 3)
    copy_img = cv2.drawContours(copy_img, [box], -1, (255,0,0), 3)
    cv2.namedWindow("Red Frame", cv2.WINDOW_NORMAL)  
    cv2.imshow("Red Frame", copy_img)
    cv2.resizeWindow("Red Frame", (800,500))
