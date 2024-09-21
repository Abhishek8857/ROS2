Overview
This project is an implementation of a Line Follower Robot, designed to autonomously follow a path using real-time image processing and control algorithms. The robot detects a red line using a camera and adjusts its velocity and direction accordingly. The system is built using ROS 2 (Humble), and OpenCV for image processing, with a modular design for detection, control, and path-following logic.

Key Features
 - Real-time Path Detection: The robot uses image processing techniques to detect and follow a red line on a surface.
 - Smooth Path-following Control: A custom control system adjusts the robot's velocity and direction based on the line's position.
 - Video Recording: Records the robot's camera feed during operation and saves the video for later analysis.
 - ROS 2 Integration: Uses ROS 2 for seamless communication between components (camera, path detection, and robot control).

Project Structure
Here’s a brief description of the main components:

 - controller.py: Contains the Controller class, which sends velocity commands to the robot based on path-following inputs.
 - detection.py: The Detection class processes the video feed, detects the line, and identifies the robot’s deviation from the center of the path.
 - follow.py: The main script combining detection and control. It processes the camera feed and controls the robot's motion. It also saves video recordings of the robot’s path.

How It Works
Path Detection
The robot's camera feed is processed as follows:

 - Gaussian Blurring: To reduce noise and smooth the image.
 - HSV Conversion: Converts the image to HSV color space to easily mask out the red line.
 - Masking: A binary mask is applied to detect the red color of the line.
 - Contour Detection: Finds the largest contour, assumed to be the path, and tracks its position relative to the center of the image.
 - Centroid Calculation: Calculates the center of the detected path to determine the robot’s deviation from the center.

Control Logic
The control system adjusts the robot’s velocity based on the detected path:

 - Left Deviation: Adjusts the robot’s angular velocity to the left.
 - Right Deviation: Adjusts the robot’s angular velocity to the right.
 - Center Alignment: Moves the robot forward in a straight line.
 - No Path Detected: The robot rotates to search for the path.

