"""" Detect color markers for px-to-unit conversion (possible future homography aswell)"""
""" Returns relation from pixel to real world unit distance """
""""Source color detection: https://agneya.medium.com/color-detection-using-python-and-opencv-8305c29d4a42"""

from main import prepareImageScaleCrop
import numpy as np
import cv2

def detectionLoop(webcam):
    _, imageFrame = webcam.read()

    # Måste ha prepareimage här för att få samma pixel conversion rate.
    imageFrame = prepareImageScaleCrop(imageFrame)

    # Convert BGR to HSV colorspace
    hsvFrame = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2HSV)

    # blue color
    blue_lower = np.array([100, 90, 100], np.uint8)
    blue_upper = np.array([120, 255, 255], np.uint8)
    # post-it ljusblå-värden: [20, 100, 200],[100, 255, 255]
    # Lim-blöta post-it ljusblå: [100, 90, 100], [120, 255, 255]

    blue_mask = cv2.inRange(hsvFrame, blue_lower, blue_upper)

    # to detect only that particular color
    kernal = np.ones((5, 5), "uint8")

    # blue color
    blue_mask = cv2.dilate(blue_mask, kernal)
    res_blue = cv2.bitwise_and(imageFrame, imageFrame, mask=blue_mask)
    cv2.imshow("Res_blue", res_blue)

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

def calcPx2mmConversion(markers, upperSideLength_mm):
    # sideLengths = np.linalg.norm(markers[0]-markers[1]) # for now only upper side
    print("Calc px2mm using points: ")
    print(f"{markers[1]} and {markers[3]}")
    upperSideLength_px = np.sqrt(
        np.power((markers[3][0] - markers[1][0]), 2) + np.power((markers[3][1] - markers[1][1]), 2))
    #print(upperSideLength_px)
    px2mm = upperSideLength_px / upperSideLength_mm
    #print(px2mm)
    return px2mm

def calcCenterPixelCoords(markers):
    x1, y1 = markers[0]
    x3, y3 = markers[2]
    x2, y2 = markers[1]
    x4, y4 = markers[3]

    # Midpoint of diagonal 1 (1-3) and diagonal 2 (2-4)
    x13 = (x1 + x3) / 2
    y13 = (y1 + y3) / 2
    x24 = (x2 + x4) / 2
    y24 = (y2 + y4) / 2

    # Average of the two midpoints = center
    x = (x13 + x24) / 2
    y = (y13 + y24) / 2
    return (x, y)

if __name__ == '__main__':
    # turn on cam
    deviceID = "/dev/video2"
    deviceID = 0
    webcam = cv2.VideoCapture(deviceID)

    while(True):
        markers = detectionLoop(webcam)

        upperSideLength_mm = 290
        # Lägg till hantering om inga markers hittas
        #px2mm = calcPx2mmConversion(markers,upperSideLength_mm)
        #print(px2mm)

        if cv2.waitKey(10) & 0xFF == ord('q'):
            print("Markers detected: ")
            print(markers)
            # Order markers
            markers = sorted(markers, key=lambda x: (x[0], x[1]))
            print("Markers sorted: ")
            print(markers)
            px2mm = calcPx2mmConversion(markers, upperSideLength_mm)
            print(px2mm)
            centerPixel = calcCenterPixelCoords(markers)
            centerMm = np.array(centerPixel)*px2mm
            print("Center: ")
            print(f"Pixel: {centerPixel} Millimeter: {centerMm}")
            webcam.release()
            cv2.destroyAllWindows()
            break

    # Calculate lengths between markers (Only upper side length)
    