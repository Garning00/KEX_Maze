# Run in N++: "D:\Program Files\Python\python.exe" -i "$(FULL_CURRENT_PATH)"

import cv2

# Choose capture device
cap = cv2.VideoCapture("/dev/video2")  # deviceID Windows: 0 Linux: "/dev/video2" (check ls in cd /dev)
# Video Capture Properties:
# https://docs.opencv.org/3.4/d4/d15/group__videoio__flags__base.html
cap.set(3, 640)  # Width
cap.set(4, 480)  # Height
cap.set(10, 150)  # Brightness

while True:
    success, img = cap.read()
    cv2.imshow("Video", img)

    #img = img[0:420, 65:580]
    scale = 0.2
    img = cv2.resize(img, (0, 0), fx=scale, fy=scale)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    cv2.imshow("Gray", img)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Motion Mask:
# https://medium.com/@itberrios6/introduction-to-motion-detection-part-1-e031b0bb9bb2

# OpenCV Cheat Sheet:
# https://www.geeksforgeeks.org/python-opencv-cheat-sheet/