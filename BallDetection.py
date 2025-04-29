import cv2
import numpy as np # for detectCircles

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
    # Settings Video02 (Top-Toy)
        # blur kzise: 13 (15 för stillbild), thresh 120 max 255, blur kzise 5, dilate 3, blur kzise 9
    img = cv2.medianBlur(img,3) # Blur får bort linjen
    #cv2.imshow("innan thresh",img)

    # Remove the dark holes using threshold and bitwise_or
    _, imgHoles = cv2.threshold(img, 100, 255, cv2.THRESH_BINARY_INV) # Thresh 100 funkar på stillbilderna men behöver 120 för video02
    #cv2.imshow("efter thresh", imgHoles)
    imgHoles = cv2.dilate(imgHoles, None, iterations=2)
    img = cv2.bitwise_or(img, imgHoles)
    #img = cv2.medianBlur(img, 3)
    #cv2.imshow("efter bitwise or", img)

    # Get marble mask
    _, img = cv2.threshold(img, 118, 255, cv2.THRESH_BINARY_INV)
    #cv2.imshow("efter marble thresh", img)

    #img = cv2.medianBlur(img,5)
    img = cv2.dilate(img, None, iterations=3)
    img = cv2.medianBlur(img, 9)
    return img

def processBlurTopToyPhoneCam(img):
    # Settings Video02 (Top-Toy)
        # blur kzise: 13 (15 för stillbild), thresh 120 max 255, blur kzise 5, dilate 3, blur kzise 9
    #cv2.imshow("test", img)
    img = cv2.medianBlur(img,13) # 15 för stillbild 13 för video02
    #cv2.imshow("innan thresh",img)
    _, img = cv2.threshold(img, 120, 255, cv2.THRESH_BINARY_INV) # Thresh 100 funkar på stillbilderna men behöver 120 för video02
    #cv2.imshow("efter thresh", img)
    img = cv2.medianBlur(img,5)
    img = cv2.dilate(img, None, iterations=3)
    img = cv2.medianBlur(img, 9)
    return img

""" Using light reflection in marble """
def processLight(img):
    #img = cv2.medianBlur(img, 1)

    # Threshold whitest parts (Light reflection)
    _, img = cv2.threshold(img, 250, 255, cv2.THRESH_BINARY)
    # Lite lägre threshold så dyker reflektion i planet upp
    # Måste hantera annat som dyker upp så som: sticker, skruvar och metallstavar!

    #cv2.imshow("processLight Thresh", img)

    # Make little bigger for circle detetction (Maybe use other method now?)
    img = cv2.dilate(img, None, iterations=5)
    img = cv2.medianBlur(img, 9)

    return img

# How to detect ball coords?
# FindContours, SimpleBlobDetector, HoughCircles

""" Detects circles hopefully well, unreliable without binary mask"""
""" inputs image to detect and image to print circles to, returns detected circles coords & radius"""
def detectCircles(img, imgOut):
    # https://www.geeksforgeeks.org/circle-detection-using-opencv-python/

    # Top-Toy, processBlur values 1, 20, 50, 10, 1, 10
    detected_circles = cv2.HoughCircles(img,
                   cv2.HOUGH_GRADIENT, 1, 30, param1 = 50,
               param2 = 10, minRadius = 1, maxRadius = 10)
    # param2 ensures better circles --> higher value (10 quite low?) 
  
    # Draw circles that are detected. 
    if detected_circles is not None: 
      
        # Convert the circle parameters a, b and r to integers. 
        detected_circles = np.uint16(np.around(detected_circles)) 
      
        for pt in detected_circles[0, :]: 
            a, b, r = pt[0], pt[1], pt[2] 
            
            #print(f"Circle at coords: ({a}, {b}) with radius: {r}")
            
            # Draw the circumference of the circle. 
            cv2.circle(imgOut, (a, b), r, (127, 0, 0), 2) # Grey ring (color not visible in binary mask)
      
            # Draw a small circle (of radius 1) to show the center. 
            cv2.circle(imgOut, (a, b), 1, (50, 0, 0), 1)

        # Display detected circles
        cv2.imshow("Detected Circles", imgOut)

        """ Add function to only return most likely to be the ball if there's more than 1 circle """
        return detected_circles[0][0] # Only outputs 1st circle for now
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
    # Resize images (Removed, process image and crop correctly beforehand)
    #scale = 0.1
    #img = cv2.resize(img, (0, 0), fx=scale, fy=scale)

    #img = processBlur(img)
    #img = processBlurTopToyPhoneCam(img)
    img = processLight(img)
    return detectCircles(img,img)


if __name__ == '__main__':
    #Image paths (/ on mac & linux and \ on PC)
    pathImgNoBall = "KEX_Bilder/Top-Toy Labyrint/Flat_noBall_Flash.jpg"
    pathImg1 = "KEX_Bilder/Top-Toy Labyrint/Flat_Ball_Flash.jpg"
    pathImgblob = "KEX_Bilder/simple_blob-2.jpg"
    pathImgTilt = "KEX_Bilder/Top-Toy Labyrint/Tilted_Ball_Flash.jpg"
    pathVideoLvl1 = "KEX_Bilder/BRIO Labyrint/Video_Lvl1.avi"
    pathLvl1Center = "KEX_Bilder/BRIO Labyrint/Lvl2_Flat_Centered.png"
    pathLvl1Tilted = "KEX_Bilder/BRIO Labyrint/Lvl2_Tilted.png"

    # Import images
    imgNoBall = cv2.imread(pathImgNoBall, cv2.IMREAD_GRAYSCALE)
    img1 = cv2.imread(pathImg1, cv2.IMREAD_GRAYSCALE)
    imgblob = cv2.imread(pathImgblob,cv2.IMREAD_GRAYSCALE)
    imgTilt = cv2.imread(pathImgTilt,cv2.IMREAD_GRAYSCALE)
    imgLvl1Center = cv2.imread(pathLvl1Center,cv2.IMREAD_GRAYSCALE)
    imgLvl1Tilted = cv2.imread(pathLvl1Tilted, cv2.IMREAD_GRAYSCALE)

    # Resize images
    scale = 0.2
    imgNoBall = cv2.resize(imgNoBall, (0, 0), fx = scale, fy = scale)
    img1 = cv2.resize(img1, (0, 0), fx = scale, fy = scale)
    imgTilt = cv2.resize(imgTilt, (0, 0), fx=scale, fy=scale)
    imgLvl1Center = cv2.resize(imgLvl1Center, (0, 0), fx=scale, fy=scale)
    imgLvl1Tilted = cv2.resize(imgLvl1Tilted, (0, 0), fx=scale, fy=scale)

    #Crop
    #imgLvl1Center = imgLvl1Center[:,21:110]

    #cv2.imshow("Tilt",imgTilt)

    #cv2.imshow("Diff-process",processDiff(imgNoBall,img1))
    cv2.imshow("Original Centered", imgLvl1Center)
    cv2.imshow("Original Tilted", imgLvl1Tilted)
    cv2.imshow("Blur-process",processBlur(imgLvl1Center))
    cv2.imshow("Light-process Centered",processLight(imgLvl1Center))
    cv2.imshow("Light-process Tilted", processLight(imgLvl1Tilted))
    detectCircles(processLight(imgLvl1Tilted),imgLvl1Tilted)

    #cv2.imshow("Blur-process Tilted", processBlur(imgLvl1Tilted))
    #print(detectCircles(processBlur(img1), img1))
    #print(detectCircles(processBlur(imgNoBall),imgNoBall))
    #print(detectCircles(processBlur(imgTilt), imgTilt))
    #print(GetBallCoords_ImageCoords(img1))
    #cv2.imshow("Detected Circles - processBlur",detectCircles(processBlur(img1),processBlur(img1)))
    #cv2.imshow("Detected Cirles - processDiff",detectCircles(processDiff(imgNoBall,img1),processDiff(imgNoBall,img1)))

    #detectBlobs(processBlur(img1),img1)
    #detectBlobs(imgblob,imgblob)

    #detectCircles(imgblob,imgblob)


    print(type(img1))


    # Wait for input and close
    cv2.waitKey(0)
    cv2.destroyAllWindows()