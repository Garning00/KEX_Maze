import numpy as np 
import cv2
from main import prepareImageScaleCrop




#path = r'captured_image_2.jpg'
path = "../KEX_Bilder/BRIO Labyrint/Lvl1_Flat_NoBall.png"

font = cv2.FONT_HERSHEY_COMPLEX 
img2 = cv2.imread(path, cv2.IMREAD_COLOR)
img = cv2.imread(path)

img = prepareImageScaleCrop(img)
img2 = prepareImageScaleCrop(img2)

cv2.waitKey(0) 
  
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) 
  
edged = cv2.Canny(gray, 80, 115) 
cv2.waitKey(0) 
  
contours, hierarchy = cv2.findContours(edged,  
    cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE) 

contour = max(contours, key = cv2.contourArea)

approx = cv2.approxPolyDP(contour, 0.0009 * cv2.arcLength(contour, True), True) 
  
# draws boundary of contours. 
cv2.drawContours(img2, [approx], 0, (0, 0, 255), 1)  

# Used to flatted the array containing 
# the co-ordinates of the vertices. 
print(approx)

n = approx.ravel()  

print(n)
i = 0



for j in n : 
    if(i % 2 == 0): 
        x = n[i] 
        y = n[i + 1] 

        # String containing the co-ordinates. 
        string = str(x) + " " + str(y)  

        if(i == 0): 
            # text on topmost co-ordinate. 
            cv2.putText(img2, "Arrow tip", (x, y), 
                            font, 0.5, (255, 0, 0))  
        else: 
            # text on remaining co-ordinates. 
            cv2.putText(img2, string, (x, y),  
                        font, 0.5, (0, 255, 0))  
    i = i + 1

# Showing the final image. 
cv2.imshow('image2', img2)  

cv2.drawContours(img, contour, -1, (0, 255, 0), 3) 
 
cv2.imshow('Contours', img) 
cv2.waitKey(0) 
cv2.destroyAllWindows() 


