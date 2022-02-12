# Thymio Project
By: Iskandar Khemakhem, Erik Börve, Maximilian van Amerongen, Romane Belda

Basics of Mobil Robotics, EPFL, 12-12-2020


## Abstract
This report presents the choices we made to achive the project goal of navigating the differential drive robot, refered to as "Thymio", in an enviorment containing known and unknown obstacles. Furthermore, certain parts of the code will be highlighted to support the made design choices.

To achieve the goal, different image processing techniques have been applied to sense Thymio's enviornment and to build up a map. Different path planning techniques were applied to the map to come up with a path that navigates the robot from its start to its goal position.
A Kalman Fitler was designed to fuse different sensors and localize the Thymio on the map.

Finally, simple motion control combined with local obstacle avoidance has been applied to achieve path following and to avoid unknown obstacles.

The resulting Motion control is displayed in the video below.


https://user-images.githubusercontent.com/81572776/153708500-784dddef-6b9c-4efc-996d-09a122e975ed.mp4

## Introduction
December 2021, a group of 4 students has kipnapped one of the famous robots, called "Thymio". Thymio is having a hard time with the students, why he is trying to escape from them and to return back to its creator Mr. Thymio. However, it turns out that this is not easy task. During his journey, Thymio will face global obstacles, students will cross his path (local obstacles) and he will be kipnapped for a second time. The question is, in this difficult circumstances, will Thymio manage to reach his goal to be reunited with its creator agian?

We wish him all the best.

![set_up](https://raw.githubusercontent.com/BorveErik/ThymioProject/main/Report_Images/set_up.jpg)


## Vision
### Image processing to indentify environment
First, taking the raw camera input and reading it as a grayscale image yields the following result.

![raw_input](https://raw.githubusercontent.com/BorveErik/ThymioProject/main/Report_Images/input_img.png)

The grayscaled images are then preprocessed in 3 steps. To preserve the edges in the initial smoothing step, a bilateral filter was applied. After smoothing, it was possible to perform a threshholding operation which was tuned based on the current lighting conditions to maximize accuracy. Finnaly, a morphological opening filter was applied to remove any potential noise and smooth edges.

![preprocess](https://raw.githubusercontent.com/BorveErik/ThymioProject/main/Report_Images/preprocess_img.png)

To then identify the environment, the contours of the image are then calculated.
To ensure that the correct contour is found, the maximum distance between the extreme points of the contour is calculated. 
The correct contour then corresponds to contour with the largest calculated distance. This gives the following,

![contours](https://raw.githubusercontent.com/BorveErik/ThymioProject/main/Report_Images/contour_img.png)

Finally, to express the environment from an appropriate perspective, a 4-point transform is computed using the corners of the contour. 
The result is a cropped and rotated frame that can be used both initially for the global pathfinding and continuously to give appropriate measurements for the kalman filter.

![warp_res](https://raw.githubusercontent.com/BorveErik/ThymioProject/main/Report_Images/warp_img.png)

### Goal and start position localization
To have a robust localization of the beautiful goal the scale invariant feature transform (SIFT) algorithm was used. Applying this to the template and input image gives a set of matching features points in each corresponding image. To get a single fixed goal position the mean position of the 10 best SIFT correspondance was calculated. Obtaining the initial position of the Thymio was done using the Aruco packages which is described further in the localization section. The final result is the following,

![img_res](https://raw.githubusercontent.com/BorveErik/ThymioProject/main/Report_Images/result_img.png)

## Global Navigation
Having the map from the vision part in an array form, we start by down scaling the map to reduce the computation time. A maximum mask is used for this purpose, i.e., for a downscaling with a factor m in the $x$-direction and a facor n in the $y$-direction, every m $\times$ n-square of pixels will be reduced to one pixel. In case there is any part of the obstacle in the mask, the resulting pixel will take the value of an obstacle.  
The -<img src="https://latex.codecogs.com/gif.latex? x " />- and $y$-coordinates of the obstacles in the scaled map are then extracted in a list and given to the path-planning.
