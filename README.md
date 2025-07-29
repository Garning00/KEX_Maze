# Self-solving Maze Board Game: A KTH Bachelor Degree Project

## About
This repo houses all code used for the project and this README describes how the code works and what functions and variables to tweak depending on setup.
The project is paired with a bachelor thesis found here ... written by Lucas Karlsson & Linus Wretman.

## Code structure
This section lists code used during development and states its purpose.
### Arduino Sketches
The following sketches can be uploaded to Arduino:
* **AccelStepperMove.ino**  
Controls steppers using AccelStepper Arduino library and allows for control using keyboard input (Linus). Used for testing.
* **Balancing.ino**  
Main code containing PID and motor control. Receives marble's position in px
from main.py together with setpoint and moves the stepper motors according to PID calculations.

### Python Ball Detection and Position Calculation
The following scripts have been used:
* **OpenCVReadCamera**  
A simple test script to show the camera
*  **StepperMovement**  
Uses Pyserial Python library to communicate with Arduino and control steppers using keyboard input. Paired with AccelStepperMove.ino Arduino Sketch.
Script must be run as admin, only tested on Linus (may need to reinstall libraries on admin Python or give access).
* **ColorDetection**  
Establishes a color mask and using contours tracks colored markers to get bounds of the game area for pixel to real world unit conversion. Adjust the lower and upper bounds to align with desired color to detect. The reference length *upperSideLength_mm* must be tweaked to give a reference from pixels to real world units.
This is the distance between the center points of the two topmost markers in the picture. This conversion rate is later plugged in manually in main.py (Only used for plotting result in mm units).
* **BallDetection**  
Contains multiple processes for creating a mask for the metal marble. The process used in the thesis is processLight which uses
the light reflection on the marble to create a mask. To avoid potential disturbance *Hough circle detection* is used to detect only
the marble from the mask and returns the circle's coordinates, all in the detectCircles function. The function GetBallCoords_ImageCoords is called
from BallVelocity.py and performs processLight and detectCircles, thus returning the circle's coordinates.
* **BallVelocity**  
The function GetVelocity_ImageCoords is called from main.py and compares to video frames and calculates the 
marble's velocity in the unit pixels/s. If ball detection fails for subsequent frames the last known position and
velocity is used.
*  **Main**  
Reads video feed, performs initial image processing and cropping and calls GetVelocity_ImageCoords.
The position is sent to Arduino together with the setpoint (dependent on input mode) using serialQuery function. Serial communication is handled using the Pyserial Python library.

### Line/Route Detection
* **Route_wFocus**  
Finds the drawn route on the maze using OpenCV canny edge detection and contour finding. Mouse-clicking on start and end point in the camera window enables sorting the data points. Final list of points is printed and can be inserted into main.py for line-following.
* **Route_algorithm**  
Used for testing finding correct parameters to detect the path. Lacks sorting of data points and inputting endpoints using mouse-click.

## Setup procedure
1. Edit Balancing.ino output limits (in stepper motor steps) and other parameters for your design and upload to Arduino.
2. Run BallDetection.py, select and tweak desired processing function until the marble is detected, try using still images. Modify GetBallCoords_ImageCoords to use chosen process.
3. If running in line-following mode: Run Route_wFocus.py. Click the camera window at the start and end location. Tweak edge and contour detection functions until the path is defined. Once functioning, the ordered list of points should be printed and can be copied to main.py for line-following.
4. Run main.py with correct frame rate variable and choose between line-following or mouse-click mode.

## Common pitfalls
### Linux and PC differences
* Filepath structures are different, use / on mac & Linux and \ on PC.
* Reading correct camera deviceID using OpenCV video capture, Linux uses something like: "/dev/video2" and a number (0) for Windows.
* Selecting correct serial port name, Linux uses something like: "/dev/ttyACM0" and for Windows: "COM4".
* Use same baudrate in Python as in the Arduino sketch.
* On Linus serial port must be granted permission using "sudo chmod a+rw /dev/ttyACM0" (replace with correct port name).
### Other
* There's currently no code to receive serial messages in Python sent from Arduino. This might then cause the program to stall if not accompanied with serial monitor in Arduino IDE (Python might be needed to run as admin to work)
