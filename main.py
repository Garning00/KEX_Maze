"""" import relevant scripts and run here """
""" sudo chmod a+rw /dev/ttyACM0 """
import cv2
from BallVelocity import GetVelocity_ImageCoords
import serial
import time
from matplotlib import pyplot
import numpy as np
from Timer import Timer

def prepareImage(img):
    img = prepareImageScaleCrop(img)
    img = prepareImage2Gray(img)
    return img

def prepareImageScaleCrop(img):
    scale = 0.5
    img = cv2.resize(img, (0, 0), fx=scale, fy=scale)
    #cv2.imshow("Source", img)

    # Crop
    # img = img[:, 25:117]
    return img

def prepareImage2Gray(img):
    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    return img

# Query has to follow correct message layout to avoid delay ()
def serialQuery(message):
    message += '\n'
    ser.write(message.encode('ascii'))
    #print("Message sent: " + message)
    #print("message sent at" + time.ctime())

def getMouseClick(event,x,y,flags,param):
    global mouseSetpoint
    if event == cv2.EVENT_LBUTTONDOWN:
        mouseSetpoint = (x,y)
        print(f"mouse click at ({x},{y})")

def getCurrentSetpoint():
    global pointIndex
    if setpointMode == "mouse":
        return mouseSetpoint
    elif setpointMode == "list":
        if pointIndex < len(pointList):
            return pointList[pointIndex]
        else:
            return None  # or hold last point

def draw_cross(img, center, size, color, thickness):
    x, y = center
    half = size // 2
    cv2.line(img, (x - half, y), (x + half, y), color, thickness)
    cv2.line(img, (x, y - half), (x, y + half), color, thickness)

if __name__ == '__main__':
    # Initialize serial
    #serialName: str = 'COM37' # Windows port name
    serialName: str = '/dev/ttyACM0'  # Linux port name
    baud: int = 19200
    ser = serial.Serial(serialName, baudrate=baud)  # , rtscts=False, dsrdtr=False)  # open serial port
    time.sleep(1)

    # Import video and parameters
    videoPath = "KEX_Bilder/Top-Toy Labyrint/Video_Flash02.mp4"
    videoPath = "KEX_Bilder/BRIO Labyrint/Video_Lvl3.avi"
    #deviceID = 0
    deviceID = "/dev/video0"
    framerate = 30
    px2mm = 0.872441051775 # Convertion rate from pixles to real world units (gathered from ColorDetection.py)
    frame_width = 640
    frame_height = 480
    scale = 0.5 # !! Not global variable, must be set to same as in prepareImageScaleCrop()
    vidResultSavePath = "KEX_Bilder/BRIO Labyrint/Results/"

    pointIndex = 0
    pointList = [(126, 205), (214, 198), (232, 185), (264, 177), (253, 132), (267, 93), (262, 68), (249, 35), (194, 24), (209, 56), (190, 85), (167, 97), (147, 120), (180, 160), (145, 172), (112, 163), (47, 188), (50, 154), (67, 131), (108, 130), (123, 102), (103, 82), (72, 79), (48, 79), (61, 84), (91, 79), (101, 84), (62, 86), (40, 78), (41, 75), (46, 38)]
    mouseSetpoint = (160,25)
    setpointMode = "mouse" #"mouse" or "list"

    errorTolerance = 10 # [px]
    timeAtPoint = 2000 # [ms]
    errorTimer = Timer(1.0)

    cap = cv2.VideoCapture(deviceID) # Choose videoPath or DeviceID
    cap.set(3, frame_width)     # Width
    cap.set(4, frame_height)     # Height
    cap.set(10, 150)    # Brightness

    # Initiate first frame to compare distance
    success, imgPast = cap.read()
    #imgPosPlot = prepareImageScaleCrop(imgPast) # Save first frame for later plot (Plane is flat as this point)
    imgPast = prepareImage(imgPast)
    imgPosPlot = imgPast # Save gray img instead

    # Save video
    vidResult = cv2.VideoWriter(f'{vidResultSavePath}Run@{time.ctime()}.avi',
                         cv2.VideoWriter_fourcc(*'MJPG'),
                         framerate, (int(frame_width*scale),int(frame_height*scale)), isColor=False) # !!! To get output you need to release, have correct dimensions and set isColor correctly
    if not vidResult.isOpened():
        print("Failed to open VideoWriter")

    lastValidImgCoords = [0,0]#None
    lastValidImgVelocity = [0, 0]

    # Initialize all positions for plotting
    savedPosx = []
    savedPosy = []
    savedRefx = []
    savedRefy = []
    savedVelx = []
    savedVely = []

    while True:
        success, img = cap.read()

        imgCurrent = prepareImage(img)

        # Write frame to file
        vidResult.write(imgCurrent)

        lastValidImgVelocity, lastValidImgCoords = GetVelocity_ImageCoords(imgPast, imgCurrent, framerate, lastValidImgCoords, lastValidImgVelocity)
        #print(f"LastVel: {lastValidImgVelocity} Last Coords: {lastValidImgCoords}")

        # Convert ImgCoords and velocity to IRL units
        vel = [e * px2mm for e in lastValidImgVelocity]
        # print(f"PosPX: {lastValidImgCoords}px")
        pos = [e * px2mm for e in lastValidImgCoords]
        #print(f"V: {vel}mm/s Pos: {pos}mm")
        #print(f"PosMM: {pos}mm")

        cv2.imshow("Video", imgCurrent)
        cv2.setMouseCallback("Video", getMouseClick)

        # Update Setpoint
        setPoint = getCurrentSetpoint()

        print(f"setPoint: {np.array(setPoint)}")
        print(f"Current pos: {lastValidImgCoords[:2]}")

        # # Increase pointIndex if ball have been stable around setpoint for 1s
        if (setpointMode == "list"):
            errorDistance = np.linalg.norm(np.array(lastValidImgCoords[:2]) - np.array(setPoint))
            if (errorDistance > errorTolerance):
                errorTimer.stop()
            else:
                print("Inside tolerance - Starting timer!")
                errorTimer.start() # Doesn't restart when calling .start()
                if errorTimer.is_done():
                    pointIndex += 1
                    print(f"Held position for 1 sec! Moving to: {pointList[pointIndex]}")
                    errorTimer.stop()


        #print(f"mouse: {mouseSetpoint[0]},{mouseSetpoint[1]} and mm {mouseSetpoint[0]*px2mm},{mouseSetpoint[1]*px2mm}")
        #print(f"Setpoint: {setPoint}")

        # Send position to arduino via serial
        # in mm:s
        #serialQuery(f"{round(pos[0],8)} {round(pos[1],8)} {round(mouseSetpoint[0]*px2mm,8)} {round(mouseSetpoint[1]*px2mm,8)}")
        # in px:s
        serialQuery(f"{lastValidImgCoords[0]} {lastValidImgCoords[1]} {setPoint[0]} {setPoint[1]}")

        # baud needs to account for 4+4+4+4 chars = 16
        # 1/baudrate * chars * 1000 = ms to send message
        # 1000 / ms per message = message per second
        # reversed: if we have 30 ms per message and baud 19200 we can have 576 chars

        savedPosx.append(lastValidImgCoords[0])
        savedPosy.append(lastValidImgCoords[1])
        savedRefx.append(setPoint[0])
        savedRefy.append(setPoint[1])
        savedVelx.append(vel[0])
        savedVely.append(vel[1])

        imgPast = imgCurrent   # Set previous frame variable for next loop

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Plot x/y-position with reference value (use this instead?? https://www.geeksforgeeks.org/how-to-update-a-plot-on-same-figure-during-the-loop/)
    pyplot.subplot(1, 2, 1)
    pyplot.plot(savedPosx, label="X")
    pyplot.plot(savedRefx, label="ref X", linestyle="--")
    pyplot.title("X-step")
    pyplot.subplot(1, 2, 2)
    pyplot.plot(savedPosy, label="Y")
    pyplot.plot(savedRefy, label="ref Y", linestyle="--")
    pyplot.title("Y-step")
    pyplot.legend()
    pyplot.show(block=False)
    pyplot.pause(0.001)
    #pyplot.clf()
    pyplot.figure(0)
    pyplot.plot(savedVelx, label="X")
    pyplot.plot(savedVely, label="Y")
    pyplot.title("speed [mm/s]")
    pyplot.legend()
    pyplot.show()

    savedPosx = [int(x) for x in savedPosx]
    savedPosy = [int(x) for x in savedPosy]
    print("X data:")
    print(savedPosx)
    print("Reference X:")
    print(savedRefx)
    print("Y data:")
    print(savedPosy)
    print("Reference Y:")
    print(savedRefy)
    print("X Speed:")
    print(savedVelx)

    # Plot image of plane with recorded position data
    imgPosPlot = cv2.cvtColor(imgPosPlot, cv2.COLOR_GRAY2BGR) # Convert back to BGR to plot colors
    for i in range(len(savedPosx)):
        cv2.circle(imgPosPlot, (savedPosx[i],savedPosy[i]), 2, (255, 0, 0), 2)
    # Plot reference positions
    for i in range(len(savedRefx)):
        if savedRefx[i] != savedRefx[i-1]:
            #cv2.circle(imgPosPlot, (savedRefx[i], savedRefy[i]), 2, (0, 255, 0), 2)
            draw_cross(imgPosPlot, (savedRefx[i], savedRefy[i]),10,(0,0,255),2)
    cv2.imshow("Recorded Positions", imgPosPlot)


    # Save film (https://www.geeksforgeeks.org/saving-a-video-using-opencv/)

    cap.release()
    vidResult.release()
    #cv2.destroyAllWindows()
    cv2.waitKey(0)