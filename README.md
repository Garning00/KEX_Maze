# Self-solving Maze Board Game: A KTH Bachelor Degree Project

## About
This repo houses all code used for the project and this readme describes how the code works and what functions and variables to tweak depending on setup.
The project is paired with a bachelor thesis found here ... written by ...

## Code structure
This section lists code used during development and states its purpose.
### Arduino Sketches
The following sketches can be uploaded to Arduino:
* **AccelStepperMove.ino**  
Controls steppers using AccelStepper Arduino library and allows for control using keyboard input (linux). Used for testing.
* **Balancing.ino**  
Main code containing PID and motor control. Receives marbles position in mm
from main.py.

### Python Ball Detection and Position Calculation
The following scripts have been used:
* **OpenCVReadCamera**  
A simple test script to show the camera
*  **StepperMovement**  
Uses Pyserial Python library to communicate with Arduino and control steppers using keyboard input. Paired with AccelStepperMove.ino Arduino Sketch.
Script must be run as admin, only tested on linux (may need to reinstall libraries on admin python or give access).
* **ColorDetection**  
Establishes a color mask and using contours tracks colored markers to get bounds of the game area for pixel to real world unit conversion.
This conversion rate is later plugged in manually in main.py.
* **BallDetection**  
Contains multiple processes for creating a mask for the metal marble. The process used in the thesis is processLight which uses
the light reflection on the marble to create a mask. To avoid potential disturbance *Hough circle detection* is used to detect only
the marble from the mask and returns the circle's coordinates, all in the detectCircles function. The function GetBallCoords_ImageCoords is called
from BallVelocity.py and performs processLight and detectCircles, thus returning the circle's coordinates.
* **BallVelocity**  
The function GetVelocity_ImageCoords is called from main.py and compares to video frames and calculates the 
marbles velocity in the unit pixels/s. If ball detection fails for subsequent frames the last know position and
velocity is used.
*  **Main**  
Reads video feed, performs initial image processing and cropping and calls GetVelocity_ImageCoords.
The position is converted to real world units using the conversion rate from ColorDetection.py (variable *pix2mm*) and then sent to Arduino
using serialQuery function. Serial communication is handled using the Pyserial Python library.

## Setup procedure
1. Upload Balancing.ino to Arduino.
2. Run BallDetection.py, select and tweak desired processing function until the marble is detected, try using still images. Modify GetBallCoords_ImageCoords to use chosen process.
3. Run ColorDetection.py. Adjust the lower and upper bounds to align with desired color to detect. The reference length *upperSideLength_mm* must be tweaked to give a reference from pixels to real world units.
This is the distance between the center points of the two top most markers in the picture. Write down the px2mm conversion rate and plug it into main.py.
4. Run main.py with correct framerate and px2mm variables.

## Common pitfalls
### Linux and PC differences
* Filepath structures are different, use / on mac & linux and \ on PC.
* Reading correct camera deviceID using OpenCV video capture, Linux uses something like: "/dev/video2" and a number (0) for Windows.
* Selecting correct serial port name, Linux uses something like: "/dev/ttyACM0" and for Windows: "COM4".
* 