# ROS-Basic-GUI
Basic Application that gives teleop and video display of a ROS Robot with a simple GUI
created for final project submission of UT Austin FRI Stream Autonomous Intelligent Robotics. 
written in C++. 
--------------------------------------------------------------------------------------
Uses the ROS framework, along with the OPENCV, GTK+, and Cairo libraries to create an 
application that offers a simple GUI for some basic ROS robot functions such as 
teleop and streaming video. 

The GUI itself is created using the GTK+ library to create the GTK application that
serves as the basis for our GUI. at the moment it only features two options, teleop
and video streaming. 

The teleop GUI allows for simple movement in 6 directions. Forward, Backwards, Left,
Right, Diagonally Right, and Diagonally Left. The robot will move in these directions
only while these buttons are pressed unlike the standard teleop_twist_keyboard. The 
GUI still allows the use of keyboard controls for the movement, and this also changes
the implementation so that the robot only moves while they keys are held down, different
than the implementation of teleop_twist_keyboard. Furthermore to simplify the control 
of the angular and linear speeds, rather than adjusting the values through key presses 
I implemented simple sliders to adjust these values.

The video streaming option utilizes the OpenCV library, along with the Cairo 2D graphic
library to take the image data from a ROS topic that is being published and creating a 
window in the application that then displays this image data. 

The idea was that by giving the application the ability to both control the robots movement
through a more simple and straightforward interface than the standard teleop_twist_keyboard
while also being able to stream video from the robot that the application could be used by
the general public to control the robots to a basic degree without needing to run mutliple
different programs to be able to do the same essential basic functions. 

CURRENT ISSUES
----------------------------------------------------------------------------------------
- Callback queues can get cluttered at this point and can lead to latency in the control 
of the robot
- The display of the image at this point will not work without a call to imshow, which is
then immediately closed. This happens each time the image data is updated, creating an almost
ghost window.


