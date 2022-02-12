import numpy as np
import cv2
from matplotlib import pyplot as plt
from myThymio import myThymio
from tdmclient import ClientAsync
import sys
import os
import global_path as gp
import time

sys.path.insert(0, os.path.join(os.getcwd(), 'Vision'))
sys.path.insert(0, os.path.join(os.getcwd(), 'GlobalNavTest'))
sys.path.insert(0, os.path.join(os.getcwd(), 'Path_planning'))

from Vision.visionMain import getNavigationMap
from Vision.visionMain import doPerspectiveTransform
from Vision.camera_pose import pose
from ekf import *

def obstacle_detect(node,thymio):
    '''
    Updates obstacle boolean in thymio object based on proximity sensors
    input:
        node object
        thymio object
    '''
    prox = node.v.prox.horizontal[0:5]
    sensorSum = prox[2] + prox[0] + prox[4]
    if sensorSum > 0:
        thymio.obstacleFound = True
    else:
        thymio.obstacleFound = False

def localObstacleAvoidance(thymio):
    '''
    Returns motor target for local obstacle avoidance.
    Based on proximity sensors and number of sample since last seen obstacle.

    Input:
        thymio object
    '''
    if thymio.obstacleFound:
        thymio.setSpeed(-25,25)
        thymio.avoidingObstacle = True
    else:
        if thymio.obstacle_iter < 3:
            thymio.setSpeed(50,50)
            thymio.obstacle_iter += 1
        else:
            thymio.setSpeed(35,-35)
            thymio.obstacle_iter = 0

            if thymio.smallestDist < 0.025:
                        print('Going back to path!')
                        thymio.setSpeed(0,0)
                        thymio.avoidingObstacle = False
                        thymio.current_point = thymio.closestPoint    # PERHAPS EDIT!!

def kalman_measure(node,thymio):
    '''
    Calculates extended kalman filter pose estimations.
    Based on pose estimation from camera and measurments of wheel speed
    input:
        node object
        thymio object
    '''
    v_l, v_r = node.v.motor.left.speed , node.v.motor.right.speed     # Read motor speed
    V_r, V_l = trans_vel(v_l), trans_vel(v_r)                         # velocity is translated to m/s
    thymio.velocity_meas(calculate_vel(V_r,V_l))                      # velocity is written to thymio

    u_l, u_r = node.v.motor.left.target, node.v.motor.right.target
    U_l, U_r = trans_vel(u_l), trans_vel(u_r)

    thymio.input_update(calculate_vel(U_r,U_l))

    EKF(thymio)

def moveThymio(thymio):
    '''
    Sets the wheel speed target in the thymio object
    '''
    move = 80
    thymio.setSpeed(move,move)

def rotateThymio(thymio):
    '''
    Rotates thymio towards angle between current and following point in global path.
    Based on extended kalman filter pose estimate.
    '''
    rot = 20
    if thymio.angle <= np.pi:
        if thymio.measAngle > thymio.angle + np.pi or thymio.measAngle < thymio.angle:
            thymio.setSpeed(-rot,rot)
        else:
            thymio.setSpeed(rot,-rot)
    if thymio.angle > np.pi:
        if thymio.measAngle > thymio.angle - np.pi and thymio.measAngle < thymio.angle:
            thymio.setSpeed(-rot,rot)
        else:
            thymio.setSpeed(rot,-rot)

def doControl(thymio):
    '''
    Swaps between moving forward and rotating towards following point in path.
    Based on difference in distance and angle
    '''
    thymio.distance2path()
    thymio.angle2point()

    print('Angle of next point: ',thymio.angle)
    print('Estimated angle: ',thymio.measAngle)
    print('Point distance:',thymio.distPoint)

    if abs(thymio.distPoint) < 0.03:        
        rotateThymio(thymio)

        if abs(thymio.anglePoint) < 0.1 or abs(thymio.anglePoint) > 2*np.pi-0.1:
            thymio.setSpeed(0,0)
            thymio.updatePoint()
    else:
        moveThymio(thymio)

def makeVideo(thymio,frame_cropped):
    '''
    Plots path visuals in transformed frame
    '''

    # Plot optinal path
    x_scale,y_scale =  gp.get_scale_factor_to_reality(frame_cropped)
    for i in range(len(thymio.rx)):
        plotPos = (int(thymio.rx[i] /x_scale * 1000),int(thymio.ry[i] / y_scale * 1000))
        edt_frame = cv2.circle(frame_cropped,
                    plotPos,
                    radius=1,
                    color = (0, 0, 255),
                    thickness = 2)
        if i == thymio.current_point:
            edt_frame = cv2.rectangle(frame_cropped,
                    (int(plotPos[0]-1),int(plotPos[1]-1)),
                    (int(plotPos[0]+1),int(plotPos[1]+1)),
                    color = (0, 255, 0),
                    thickness = 3)

    # Plot current position of thymio
    plotPos = (int(thymio.pos_est[0][0] /x_scale * 1000),int(thymio.pos_est[0][1] / y_scale * 1000))
    edt_frame = cv2.circle(frame_cropped,
                    plotPos,
                    radius = 7,
                    color = (0,255, 0),
                    thickness= 2)

    # Plot all past position of the thymio
    thymio.storePath(plotPos[0],plotPos[1])
    for i in range(1,len(thymio.xPlot)):
        plotPos = (int(thymio.xPlot[i]),int(thymio.yPlot[i]))
        edt_frame = cv2.circle(frame_cropped,
                    plotPos,
                    radius = 1,
                    color = (255,0 , 0),
                    thickness= 2)

    return edt_frame
        

with ClientAsync() as client:
    async def main():
        with await client.lock() as node:
            # Sampling time: 
            delta_t = 0.2
            # Initialize camera object
            cap = cv2.VideoCapture(0)
            fig = plt.figure()
            if (cap.isOpened() == False):
                print("Unable to read camera feed")
            frame = cap.read()[1]
            plt.imshow(frame)
            plt.show()

            # Get initial vision for navigation 
            globalMap,goal,start,R = getNavigationMap(frame)
            frame_cropped = doPerspectiveTransform(frame,R)
            startPos = np.array([pose(frame_cropped)])
            startVel = np.zeros((1,2))
            thymio = myThymio(startPos ,startVel,initialize_kalman(),delta_t)

            # Initialize object for video capture
            frame_height = len(frame_cropped)
            frame_width = len(frame_cropped[0])
            out = cv2.VideoWriter('thymioVid.avi',cv2.VideoWriter_fourcc('M','J','P','G'), 5, (frame_width,frame_height))

            # Get initial vision for navigation
            globalMap,goal,start,R = getNavigationMap(frame)

            # Get and store path planning
            rx, ry, ryaw = gp.path_planning(globalMap, start, goal, thymio.delta_t, 50)
            thymio.savePath(rx,ry)
            N_points = len(rx)

            # Run program until Thyimo is sufficiently close to the goal
            i = 0
            while True:
                print('i',i)
                print('Pos est:',thymio.pos_est)
                print('Meas:',thymio.pos_meas)
                t_start = time.time()
                
                # Wait for variables 
                await node.wait_for_variables({"prox.horizontal"})
                await node.wait_for_variables({"motor.left.speed"})
                await node.wait_for_variables({"motor.right.speed"})

                # Read measurements and do kalman filtering
                frame = cap.read()[1]
                frame_cropped = doPerspectiveTransform(frame,R)
                thymio.camera_meas(pose(frame_cropped))
                
                # Save to video
                edt_frame = makeVideo(thymio,frame_cropped)
                out.write(edt_frame)

                
                # Get kalman filter pose update
                kalman_measure(node,thymio)

                # Check sensors
                obstacle_detect(node,thymio)
                thymio.minDist2path()

                # Choose appropriate navigation
                if thymio.obstacleFound or thymio.avoidingObstacle:
                    localObstacleAvoidance(thymio)
                else:
                    print('Following global path')
                    doControl(thymio)

                # Increment sample time with compilation time to ensure constant sampling frequency
                t_end = time.time()
                t_delta = t_end-t_start
                
                await node.set_variables({"motor.left.target":[thymio.motorLeft],
                        "motor.right.target":[thymio.motorRight]})
                await client.sleep(thymio.delta_t-t_delta)

                i += 1
                # Exit condition
                if thymio.current_point >= N_points-1:
                    await node.set_variables({"motor.left.target":[0],
                        "motor.right.target":[0]})
                    break
                cv2.imwrite("out.jpg", edt_frame)
            
            cap.release()
            out.release()
            cv2.destroyAllWindows()
    client.run_async_program(main)
