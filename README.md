# C++ and OOPS
Project demonstrating C++ knowledge.



# Description
The objective of the code is to do visual servoing using VISP. The code takes in images and finds circles in it using hough circle transforms and then using VISP we calculate the velocity to move closer towards the circle center. The velocity is in camera frame we use a suitable extrinsic matrix to convert velocity from camera frame to fixed robot frame. The code also uses a intrinsic parameters to distort the incoming images.IOU condition is used to terminate the code when the robot moves closer to the circle centroid. The extrinsic, intrinsic, IOU threshold value , radius range to detect circles are kept in a separate text file. The code will take the values in the text file. Due to contract issues the input for testing the code cannot be shared, I have isntead attached the link to the code working video. This project is done on Ubuntu. 


Video of working:https://drive.google.com/file/d/1fUZNtzLUzDNtRdnTvWbDX7HO-I6FTyhE/view?usp=sharing

Required packages: VISP software
https://visp-doc.inria.fr/doxygen/visp-daily/tutorial-install-ubuntu-package.html

# Instruction:
- Install VISP
- Create a visp workspace
- Create a build folder in the workspace
- Compile the code in the workspace
- Run it 
- Caution: emc1.yaml, config.txt,location.txt must be put in the build folder.
