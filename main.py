"""" import relevant scripts and run here """
import cv2
from BallVelocity import GetVelocity_ImageCoords

if __name__ == '__main__':
    videoPath = "KEX_Bilder/Top-Toy Labyrint/Video_Flash.mp4"

    cap = cv2.VideoCapture(videoPath)
    cap.set(3, 640)     # Width
    cap.set(4, 480)     # Height
    cap.set(10, 150)    # Brightness

    while True:
        success, img = cap.read()
        #Rescale
        scale = 0.1
        img = cv2.resize(img, (0, 0), fx=scale, fy=scale)
        #Crop
        img = img[:, 20:300]

        imgGray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        frameCurrent = imgGray

        #GetVelocity_ImageCoords()

        cv2.imshow("Video", img)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break