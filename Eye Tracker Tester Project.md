# Eye Tracker Testing Device
![gantry](https://github.com/lukajuci/Engineering-Portfolio/blob/main/images/gantry.png)

## About
The purpose of this project was to creat a robot with eyes that can be used to test the efficacy of different eye tracker devices. The inspiration behind this project was to find the best eye trackers that can be used for ALS patients. This was my bachelor's capstone project in partnership with Microsoft, and my team was the 5th group to work on the project. My contribution to the project includes programming and tuning the servos that articulate the eyes, designing a laser mount to fit over the eyes, and coming up with a novel laser calibration procedure so that we could ensure the lasers were concentric with the gaze of the robots eyes.

## Laser Calibration
We wanted to put lasers over the eyes so that we could see where the eyes were actually looking and then validate that the movement was accurate and repeatable. Without access to a laser lab or calibration equipment I came up with a new method using the lathe and 4 jaw chuck available in our machine shop. The procedure is as follows:

1. The eye enclosure with the laser mount attached are chucked up. 
2. A dial gauge is positioned so that it touches the part of the eye enclosure that is gripped by the chuck.
3. The lathe is rotated and the jaws of the chuck are adjusted until the dial gauge does not move more than 0.001" for our case. This ensures that the eye enclosure is concentric with the lathe so that when the lathe is rotated, the gaze of the eye remains fixed in place.
4. Power on the lasers and place something on the other end of the lathe to shine the laser onto. We used graph paper because of the grid lines making adjustment easier.
5. Rotate the lathe and adjust the set screws on the laser mount until the laser is fixed on a single point for a full rotation. Now the laser is conecntric with the gaze of the eyes.

Note: This assumes that the eye enclosure is constructed so that all components are concentric. We did not know what the design tolerances were for the eye enclosure

Obviously this isn't perfect, but I came up with the process by drawing upon my previous experience in the machine shop and I was proud of that. We still managed to achieve movement accuracy of 0.1 degrees, or about +/-1.1 mm which wouldn't have been repeatable enough for validation if our lasers weren't pretty concentric with the eye gaze.

![chucked up](https://github.com/lukajuci/Engineering-Portfolio/blob/main/images/calibration1.jpg)
![graph paper](https://github.com/lukajuci/Engineering-Portfolio/blob/main/images/calibration2.jpg)

## Coding and Tuning
Since we were the 5th team to work on this project a low of the code for calibrating and moving the robot was already written which was awesome, except for the fact that it didn't work. Between libraries getting updated, components being replaced, and some questionable code the robot was completely non function when we got it. The basic structure of the code was a state machine where a user could chose to calibrate the robot, tell it to look somewhere, settings, and the main menu. The robot is able to move its "shoulders" on the x, y, and z axis while the "eyes" had pitch and yaw freedom of momvent.

After fixing all the broken library instances I created new transformation matrices for controlling the movement of the eyes in reference to where they should be looking on the screen. With the updated matrices and some modification to the existing code I got the eyes 
