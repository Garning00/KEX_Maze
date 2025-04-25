"""" import relevant scripts and run here """
""" sudo chmod a+rw /dev/ttyACM0 """
import cv2
from BallVelocity import GetVelocity_ImageCoords
import serial
from time import sleep

def prepareImage(img):
    img = prepareImageScaleCrop(img)
    img = prepareImage2Gray(img)
    return img

def prepareImageScaleCrop(img):
    scale = 1
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

if __name__ == '__main__':
    # Initialize serial
    #serialName: str = 'COM5' # Windows port name
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
    px2mm = 1.738 # Convertion rate from pixles to real world units (gathered from ColorDetection.py)

    cap = cv2.VideoCapture(deviceID) # Choose videoPath or DeviceID
    cap.set(3, 640)     # Width
    cap.set(4, 480)     # Height
    cap.set(10, 150)    # Brightness

    # Initiate first frame to compare distance
    success, imgPast = cap.read()
    imgPast = prepareImage(imgPast)

    lastValidImgCoords = [0,0]#None
    lastValidImgVelocity = [0, 0]
    

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
        vel = lastValidImgCoords*px2mm
        pos = lastValidImgCoords*px2mm
        #print(f"V: {vel}mm/s Pos: {pos}mm")
        print(f"Pos: {pos}mm")

        # Send position to arduino via serial
        serialQuery(f"{pos[0]} {pos[1]}")

        cv2.imshow("Video", imgCurrent)

        imgPast = imgCurrent   # Set previous frame variable for next loop

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break