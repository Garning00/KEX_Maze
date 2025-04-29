from symbol import pass_stmt

import cv2
from BallDetection import GetBallCoords_ImageCoords

""" Calculates ball velocity in image pixel speed between two image inputs """
""" OBS if no ball is detected, adjust so it takes missing frames into account """
""" For missing frames use last known position and velocity """
def GetVelocity_ImageCoords(frame1,frame2,framerate, lastValidCoords, lastValidVelocity):
    coords1 = GetBallCoords_ImageCoords(frame1)
    coords2 = GetBallCoords_ImageCoords(frame2)

    # Use last valid coordinates/velocity if missing
    if coords1 is None:
        coords1 = lastValidCoords
    if coords2 is None:
        return lastValidVelocity, coords1

    # initial frame
    #if coords1 is None:
        #raise TypeError
    # Check if one frame is missing
    # if coords2 is None:
    #     raise SystemExit(coords1)

    # Coords in unsigned uint16, cannot subtract, cast to int
    dx = int(coords2[0])-int(coords1[0])
    dy = int(coords2[1])-int(coords1[1])
    #print(f"(dx,dy): ({dx},{dy})")
    vx = dx * framerate
    vy = dy * framerate

    # Debug draw velocity
    cv2.line(frame2, (coords1[0], coords1[1]),(coords2[0], coords2[1]),(127,0,0),2)
    #cv2.imshow("Velocity", frame2)

    return [vx, vy], coords2 # Returns both velocity and last known position (OBS speed in pixels/s)

if __name__ == '__main__':
    pathImg1 = "KEX_Bilder/Top-Toy Labyrint/Flat_Ball_Flash.jpg"
    pathImg2 = "KEX_Bilder/Top-Toy Labyrint/Tilted_Ball_Flash.jpg"
    img1 = cv2.imread(pathImg1, cv2.IMREAD_GRAYSCALE)
    img2 = cv2.imread(pathImg2, cv2.IMREAD_GRAYSCALE)
    print(GetBallCoords_ImageCoords(img1))
    print(GetBallCoords_ImageCoords(img2))
    print(GetVelocity_ImageCoords(img1,img2,30))

    cv2.waitKey(0)
    cv2.destroyAllWindows()