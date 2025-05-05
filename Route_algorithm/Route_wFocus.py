import numpy as np 
import cv2
from main import prepareImage

def getMouseClick(event,x,y,flags,param):
    global mouseX,mouseY
    if event == cv2.EVENT_LBUTTONDOWN:
        mouseX,mouseY = x,y
        print(f"mouse click at ({x},{y})")

def getClosestPointIndex(points,target):
    distances = np.linalg.norm(points - target, axis=1)
    min_index = np.argmin(distances)
    closest_point = points[min_index]
    min_dist = distances[min_index]
    print(f"Closest point to {target} is {closest_point} with distance {min_dist}")
    return min_index

#path = r'captured_image.jpg'

deviceID = '/dev/video2'
cap = cv2.VideoCapture(deviceID)

mouseX, mouseY = -1,-1
mouseXPrev, mouseYPrev = -1,-1
mouseClicks = []
# Select start points
while True:
    ret, img = cap.read()
    imgSelectPoints = prepareImage(img)
    cv2.imshow("Choose Start/End Point", imgSelectPoints)
    cv2.setMouseCallback("Choose Start/End Point", getMouseClick)

    if mouseX != mouseXPrev or mouseY != mouseYPrev:
        mouseClicks.append((mouseX,mouseY))
        #print(mouseClicks)

        cv2.circle(imgSelectPoints, (mouseX, mouseY), 3, (255, 0, 0), 3)
        # cirkel funkar ej??

        mouseXPrev = mouseX
        mouseYPrev = mouseY

    if cv2.waitKey(1) & 0xFF == ord('q'):
        cv2.destroyAllWindows()
        break

cap.release()

start_target = np.array(mouseClicks[0])
finish_target = np.array(mouseClicks[1])
print(f"Targets selected: start={start_target} finish={finish_target}")

font = cv2.FONT_HERSHEY_COMPLEX 
#img2 = cv2.imread(path, cv2.IMREAD_COLOR)
#img = cv2.imread(path)

img = prepareImage(img)
img2 = img

cv2.waitKey(0) 
  
#gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
#gray = prepareImage(img)
gray = img
  
edged = cv2.Canny(gray, 80, 115) # Might need to adjust parameters to detect line
cv2.waitKey(0) 
  
contours, hierarchy = cv2.findContours(edged,  
    cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE) 

contour = max(contours, key = cv2.contourArea)

approx = cv2.approxPolyDP(contour, 0.00001 * cv2.arcLength(contour, True), True)
# epsilon can be adjusted for finer line approximation (lower = more points)

coords = approx.reshape(-1, 2)

# Choose start point and end point
#start_target = np.array([290, 39]) #290, 39 Lvl 1; 322, 53 Lvl 2
#finish_target = np.array([256, 414]) #256, 414 Lvl 1; 292 425 Lvl 2

# Find nearest distance from mouse click targets to points on actual line
start_idx = getClosestPointIndex(coords,start_target)
finish_idx = getClosestPointIndex(coords,finish_target)


# Lägg till att man kan klicka i ett fönster och hittar den närmaste punkt på linje som blir start/slut-punkt

"""
# Kan ha flera närliggande punkter, ta bara första:
matches = np.where((coords == start_target).all(axis=1))[0]
if matches.size > 0:
    start_idx = matches[0]  # safely extract the first match
else:
    raise ValueError("start_target not found in coords")

finish_idx = int(np.where((coords == finish_target).all(axis=1))[0])
"""

# Check distance from Arrow point to start point --> order list of points
if start_idx > finish_idx:
    n = coords[finish_idx:start_idx+1]
else: 
    n = coords[start_idx:finish_idx+1]


n_1 = n.reshape(-1, 1, 2)

# draws boundary of contours. 
#cv2.drawContours(img2, [coords], 0, (0, 0, 255), 1)  
cv2.drawContours(img2, n_1, 0, (0, 0, 255), 1)

# Used to flatted the array containing 
# the co-ordinates of the vertices. 
n = n.ravel()

#print(n)
i = 0

x_all = []
y_all = []

# 34 är antalet steg GÖR VARIABEL
for i in range(0, len(n), 34): #80 (Lvl 1); 40 (Lvl 3)
    x = n[i]
    x_all.append(x)
    y = n[i + 1]
    y_all.append(y)
    string = f"{x},{y}"
    cv2.putText(img2, string, (x, y), font, 0.5, (0, 255, 0), 1)

print(x_all)

# Showing the final image. 
cv2.imshow('image2', img2)  

cv2.drawContours(img, contour, -1, (0, 255, 0), 3) 

cv2.imshow('Contours', img) 
cv2.waitKey(0) 
cv2.destroyAllWindows() 