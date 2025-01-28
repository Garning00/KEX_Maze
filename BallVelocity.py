import cv2
from BallDetection import GetBallCoords_ImageCoords

if __name__ == '__main__':
    pathImg1 = "KEX_Bilder/Top-Toy Labyrint/Flat_Ball_Flash.jpg"
    img1 = cv2.imread(pathImg1, cv2.IMREAD_GRAYSCALE)
    print(GetBallCoords_ImageCoords(img1))