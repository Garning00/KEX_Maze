"""" Detect color markers for px-to-unit conversion (possible future homography aswell)"""
""""Source: https://agneya.medium.com/color-detection-using-python-and-opencv-8305c29d4a42"""

import numpy as np
import cv2

def detectionLoop(webcam):
    _, imageFrame = webcam.read()

    # Convert BGR to HSV colorspace
    hsvFrame = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2HSV)

    # green color
    green_lower = np.array([40, 25, 25], np.uint8)
    green_upper = np.array([80, 255, 255], np.uint8)
    green_mask = cv2.inRange(hsvFrame, green_lower, green_upper)

    # blue color
    blue_lower = np.array([100, 90, 100], np.uint8)
    blue_upper = np.array([120, 255, 255], np.uint8)
    # post-it ljusblå-värden: [20, 100, 200],[100, 255, 255]

    blue_mask = cv2.inRange(hsvFrame, blue_lower, blue_upper)

    # to detect only that particular color
    kernal = np.ones((5, 5), "uint8")

    # green color
    green_mask = cv2.dilate(green_mask, kernal)
    res_green = cv2.bitwise_and(imageFrame, imageFrame, mask=green_mask)
    cv2.imshow("Res_green", res_green)

    # blue color
    blue_mask = cv2.dilate(blue_mask, kernal)
    res_blue = cv2.bitwise_and(imageFrame, imageFrame, mask=blue_mask)
    cv2.imshow("Res_blue", res_blue)

    # Creating contour to track green color
    contours, hierarchy = cv2.findContours(green_mask,
                                           cv2.RETR_TREE,
                                           cv2.CHAIN_APPROX_SIMPLE)

    for pic, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        if (area > 6500 and area < 7000):
            x, y, w, h = cv2.boundingRect(contour)
            imageFrame = cv2.rectangle(imageFrame, (x, y),
                                       (x + w, y + h),
                                       (0, 255, 0), 2)
            center = (x+round(w/2),y+round(h/2))
            cv2.circle(imageFrame, center, 2, (127, 0, 0), 2)

            cv2.putText(imageFrame, "Green Colour", (x, y),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1.0, (0, 255, 0))

    # Creating contour to track blue color
    contours, hierarchy = cv2.findContours(blue_mask,
                                           cv2.RETR_TREE,
                                           cv2.CHAIN_APPROX_SIMPLE)

    # Variable to store marker positions
    markerCenters = []

    for pic, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        if (area > 300 and area < 400):
            x, y, w, h = cv2.boundingRect(contour)
            imageFrame = cv2.rectangle(imageFrame, (x, y),
                                       (x + w, y + h),
                                       (255, 0, 0), 2)

            center = (x + round(w / 2), y + round(h / 2))
            markerCenters.append(center)
            cv2.circle(imageFrame, center, 2, (127, 0, 0), 2)

            cv2.putText(imageFrame, "Blue Colour", (x, y),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1.0, (255, 0, 0))

    # show final detection
    cv2.imshow("Color Detection", imageFrame)
    return markerCenters

if __name__ == '__main__':
    # turn on cam
    deviceID = "/dev/video2"
    webcam = cv2.VideoCapture(deviceID)

    while(True):
        markers = detectionLoop(webcam)

        if cv2.waitKey(10) & 0xFF == ord('q'):
            print("Markers detected: ")
            print(markers)
            webcam.release()
            cv2.destroyAllWindows()
            break

    # Calculate lengths between markers
    # Order markers
    print("Markers sorted: ")
    markers = sorted(markers, key=lambda x: (x[0], x[1]))
    print(markers)
    #sideLengths = np.linalg.norm(markers[0]-markers[1]) # for now only upper side
    upperSideLength_px = np.sqrt(np.power((markers[1][0]-markers[0][0]),2) + np.power((markers[1][1]-markers[0][1]),2))
    print(upperSideLength_px)
    upperSideLength_mm = 290
    px2mm = upperSideLength_px/upperSideLength_mm
    print(px2mm)