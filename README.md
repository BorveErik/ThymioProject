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

![alt_text](https://raw.githubusercontent.com/BorveErik/ThymioProject/main/Report_Images/set_up.jpg)
