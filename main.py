"""" import relevant scripts and run here """
import cv2
from BallVelocity import GetVelocity_ImageCoords

def prepareImage(img):
    scale = 0.1
    img = cv2.resize(img, (0, 0), fx=scale, fy=scale)
    # Crop
    img = img[24:200, 88:305]

    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    return img

if __name__ == '__main__':
    videoPath = "KEX_Bilder/Top-Toy Labyrint/Video_Flash02.mp4"

    cap = cv2.VideoCapture(videoPath)
    cap.set(3, 640)     # Width
    cap.set(4, 480)     # Height
    cap.set(10, 150)    # Brightness

    # Initiate first frame to compare distance
    success, imgPast = cap.read()
    imgPast = prepareImage(imgPast)

    lastValidImgCoords = None
    lastValidImgVelocity = [0, 0]

    while True:
        success, img = cap.read()

        imgCurrent = prepareImage(img)

        lastValidImgVelocity, lastValidImgCoords = GetVelocity_ImageCoords(imgPast, imgCurrent, 60, lastValidImgCoords, lastValidImgVelocity)
        print(f"LastVel: {lastValidImgVelocity} Last Coords: {lastValidImgCoords}")
        # try:
        #     velImgCoords = GetVelocity_ImageCoords(imgPast,imgCurrent,60)
        #     #print(GetVelocity_ImageCoords(imgPast,imgCurrent,60))
        # except TypeError:
        #     print("TypeError")
        # except SystemExit as e:
        #     print("SystemExit raised: " + str(e.args[0]))

        cv2.imshow("Video", imgCurrent)

        imgPast = imgCurrent   # Set previous frame variable for next loop

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break