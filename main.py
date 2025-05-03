"""" import relevant scripts and run here """
import time

""" sudo chmod a+rw /dev/ttyACM0 """
import cv2
from BallVelocity import GetVelocity_ImageCoords
import serial
from time import sleep
from matplotlib import pyplot
import numpy as np

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
    print("Message sent: " + message)
    #print("message sent at" + time.ctime())

def getMouseClick(event,x,y,flags,param):
    global mouseX,mouseY
    if event == cv2.EVENT_LBUTTONDOWN:
        mouseX,mouseY = x,y
        print(f"mouse click at ({x},{y})")

if __name__ == '__main__':
    # Initialize serial
    #serialName: str = 'COM37' # Windows port name
    serialName: str = '/dev/ttyACM0'  # Linux port name
    baud: int = 19200
    ser = serial.Serial(serialName, baudrate=baud)  # , rtscts=False, dsrdtr=False)  # open serial port
    sleep(1)

    # Import video and parameters
    videoPath = "KEX_Bilder/Top-Toy Labyrint/Video_Flash02.mp4"
    videoPath = "KEX_Bilder/BRIO Labyrint/Video_Lvl3.avi"
    #deviceID = 0
    deviceID = "/dev/video2"
    framerate = 30
    px2mm = 0.872441051775 # Convertion rate from pixles to real world units (gathered from ColorDetection.py)

    cap = cv2.VideoCapture(deviceID) # Choose videoPath or DeviceID
    cap.set(3, 640)     # Width
    cap.set(4, 480)     # Height
    cap.set(10, 150)    # Brightness

    # Initiate first frame to compare distance
    success, imgPast = cap.read()
    # Save first frame for later plot (Plane is flat as this point)
    imgPosPlot = prepareImageScaleCrop(imgPast)
    imgPast = prepareImage(imgPast)

    lastValidImgCoords = [0,0]#None
    lastValidImgVelocity = [0, 0]

    mouseX, mouseY = 160,25


    # save all positions for plotting
    savedPosx = []
    savedPosy = []
    savedRefx = []
    savedRefy = []
    savedVelx = []
    savedVely = []
    # PLOT

    while True:
        success, img = cap.read()

        imgCurrent = prepareImage(img)

        lastValidImgVelocity, lastValidImgCoords = GetVelocity_ImageCoords(imgPast, imgCurrent, framerate, lastValidImgCoords, lastValidImgVelocity)
        #print(f"LastVel: {lastValidImgVelocity} Last Coords: {lastValidImgCoords}")
        # try:
        #     velImgCoords = GetVelocity_ImageCoords(imgPast,imgCurrent,60)
        #     #print(GetVelocity_ImageCoords(imgPast,imgCurrent,60))
        # except TypeError:
        #     print("TypeError")
        # except SystemExit as e:
        #     print("SystemExit raised: " + str(e.args[0]))

        # Convert ImgCoords and velocity to IRL units
        #vel = lastValidImgVelocity*px2mm
        #print(f"PosPX: {lastValidImgCoords}px")
        vel = [e * px2mm for e in lastValidImgVelocity]
        #pos = lastValidImgCoords*px2mm
        pos = [e * px2mm for e in lastValidImgCoords]
        #print(f"V: {vel}mm/s Pos: {pos}mm")
        #print(f"PosMM: {pos}mm")

        cv2.imshow("Video", imgCurrent)
        cv2.setMouseCallback("Video", getMouseClick)

        #print(f"mouse: {mouseX},{mouseY} and mm {mouseX*px2mm},{mouseY*px2mm}")
        # Send position to arduino via serial
        # in mm:s
        #serialQuery(f"{round(pos[0],8)} {round(pos[1],8)} {round(mouseX*px2mm,8)} {round(mouseY*px2mm,8)}")
        # in px:s
        serialQuery(f"{lastValidImgCoords[0]} {lastValidImgCoords[1]} {mouseX} {mouseY}")

        # baud needs to account for 4+4+4+4 chars = 16
        # 1/baudrate * chars * 1000 = ms to send message
        # 1000 / ms per message = message per second
        # reversed: if we have 30 ms per message and baud 19200 we can have 576 chars

        savedPosx.append(lastValidImgCoords[0])
        savedPosy.append(lastValidImgCoords[1])
        savedRefx.append(mouseX)
        savedRefy.append(mouseY)
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
    for i in range(len(savedPosx)):
        cv2.circle(imgPosPlot, (savedPosx[i],savedPosy[i]), 2, (0, 0, 255), 2)
    cv2.imshow("Recorded Positions", imgPosPlot)

    # Save film?

    cv2.waitKey(0)