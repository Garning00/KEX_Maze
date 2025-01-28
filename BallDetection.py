import cv2
import numpy as np # for detectCircles
from chardet import detect
from google.protobuf.struct_pb2 import NullValue

""" mask with difference from plane without ball present, requires homography to account for tilting plane """
def processDiff(img1, img2):
    difference = cv2.subtract(img1,img2)
    blur = cv2.medianBlur(difference, 5)    # Kernelsize >= 5 results in no noise after threshold
    _, thresh = cv2.threshold(blur, 100, 255, cv2.THRESH_BINARY)
    dilated = cv2.dilate(thresh, None, iterations=1)
    threshBlur = cv2.medianBlur(dilated, 3)
    
    """
    adThresh = cv2.adaptiveThreshold(blur, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,\
            cv2.THRESH_BINARY_INV, 11, 3)
    bitxor = cv2.bitwise_xor(img1,imgNoBall, mask = None)
    """
    
    # Display images
    #cv2.imshow("Clean",img1)
    #cv2.imshow("With Ball",img2)
    
    #cv2.imshow("Diff",difference)
    #cv2.imshow("Blur",blur)
    #cv2.imshow("Thresh",thresh)
    #cv2.imshow("Dilated",dilated)
    #cv2.imshow("Thresh Blurred",threshBlur)
    
    #cv2.imshow("Bitwise xor",bitxor)
    #cv2.imshow("Adaptive Threshold",adThresh)
    
    return threshBlur

""" Can mask without reference and homography """
def processBlur(img):
    img = cv2.medianBlur(img,15)
    _, img = cv2.threshold(img, 90, 255, cv2.THRESH_BINARY_INV)
    img = cv2.medianBlur(img,3)
    img = cv2.dilate(img, None, iterations=1)
    return img

# How to detect ball coords?
# FindContours, SimpleBlobDetector, HoughCircles

""" Detects circles hopefully well, unreliable without binary mask"""
""" inputs image to detect and image to print circles to, returns detected circles coords & radius"""
def detectCircles(img, imgOut):
    # https://www.geeksforgeeks.org/circle-detection-using-opencv-python/
    detected_circles = cv2.HoughCircles(img,  
                   cv2.HOUGH_GRADIENT, 1, 20, param1 = 50, 
               param2 = 10, minRadius = 3, maxRadius = 10)
    # param2 ensures better circles --> higher value (10 quite low?) 
  
    # Draw circles that are detected. 
    if detected_circles is not None: 
      
        # Convert the circle parameters a, b and r to integers. 
        detected_circles = np.uint16(np.around(detected_circles)) 
      
        for pt in detected_circles[0, :]: 
            a, b, r = pt[0], pt[1], pt[2] 
            
            print(f"Circle at coords: ({a}, {b}) with radius: {r}")
            
            # Draw the circumference of the circle. 
            cv2.circle(imgOut, (a, b), r, (127, 0, 0), 2) # Grey ring (color not visible in binary mask)
      
            # Draw a small circle (of radius 1) to show the center. 
            cv2.circle(imgOut, (a, b), 1, (50, 0, 0), 1)

        # Display detected circles
        cv2.imshow("Detected Circles", imgOut)
        return detected_circles
    else:
        print("No circles detected.")
        return None

""" Have not gotten this to work yet"""
def detectBlobs(img,imgOut):
    # Initialize parameter setting using cv2.SimpleBlobDetector 
    params = cv2.SimpleBlobDetector_Params() 
      
    # Set Area filtering parameters 
    params.filterByArea = True
    params.minArea = 30
      
    # Set Circularity filtering parameters 
    params.filterByCircularity = True 
    params.minCircularity = 0.7
      
    # Set Convexity filtering parameters 
    params.filterByConvexity = True
    params.minConvexity = 0.2
          
    # Set inertia filtering parameters 
    params.filterByInertia = True
    params.minInertiaRatio = 0.01
      
    # Create a detector with the parameters 
    detector = cv2.SimpleBlobDetector_create(params) 
          
    # Detect blobs 
    keypoints = detector.detect(img) 
      
    # Draw blobs on our image as red circles 
    blank = np.zeros((1, 1))  
    blobs = cv2.drawKeypoints(imgOut, keypoints, blank, (0, 0, 255), 
                              cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS) 
      
    number_of_blobs = len(keypoints) 
    text = "Number of Circular Blobs: " + str(len(keypoints)) 
    cv2.putText(blobs, text, (0, 10), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 100, 255), 1)
    
    # Show blobs 
    cv2.imshow("Filtering Circular Blobs Only", blobs)

"""" Returns ball center coords relative to the image pixel coordinate system"""
def GetBallCoords_ImageCoords(img):
    # Resize images
    scale = 0.1
    img = cv2.resize(img, (0, 0), fx=scale, fy=scale)

    img = processBlur(img)
    return detectCircles(img,img)


if __name__ == '__main__':
    #Image paths (/ on mac & linux and \ on PC)
    pathImgNoBall = "KEX_Bilder/Top-Toy Labyrint/Flat_noBall_Flash.jpg"
    pathImg1 = "KEX_Bilder/Top-Toy Labyrint/Flat_Ball_Flash.jpg"
    pathImgblob = "KEX_Bilder/simple_blob-2.jpg"

    # Import images
    imgNoBall = cv2.imread(pathImgNoBall, cv2.IMREAD_GRAYSCALE)
    img1 = cv2.imread(pathImg1, cv2.IMREAD_GRAYSCALE)
    imgblob = cv2.imread(pathImgblob,cv2.IMREAD_GRAYSCALE)


    # Resize images
    scale = 0.1
    imgNoBall = cv2.resize(imgNoBall, (0, 0), fx = scale, fy = scale)
    img1 = cv2.resize(img1, (0, 0), fx = scale, fy = scale)

    cv2.imshow("Diff-process",processDiff(imgNoBall,img1))
    cv2.imshow("Blur-process",processBlur(img1))
    print(detectCircles(processBlur(img1), img1))
    print(detectCircles(processBlur(imgNoBall),imgNoBall))
    print(GetBallCoords_ImageCoords(img1))
    #cv2.imshow("Detected Circles - processBlur",detectCircles(processBlur(img1),processBlur(img1)))
    #cv2.imshow("Detected Cirles - processDiff",detectCircles(processDiff(imgNoBall,img1),processDiff(imgNoBall,img1)))

    #detectBlobs(processBlur(img1),img1)
    #detectBlobs(imgblob,imgblob)

    #cv2.imshow("blob circles detected",detectCircles(imgblob,imgblob))


    print(type(img1))


    # Wait for input and close
    cv2.waitKey(0)
    cv2.destroyAllWindows()