#!/usr/bin/python
# -*- coding: utf-8 -*-

from math import *
import numpy as np
import probabilistic_lib.functions as funcs
import rospy

#============================================================================
class EKF_SLAM(object):
    '''
    Class to hold the whole EKF-SLAM.
    '''
    
    #========================================================================
    def __init__(self, x0,y0,theta0, odom_lin_sigma, 
                 odom_ang_sigma, meas_rng_noise, meas_ang_noise):
        '''
        Initializes the ekf filter
        room_map : an array of lines in the form [x1 y1 x2 y2]
        num      : number of particles to use
        odom_lin_sigma: odometry linear noise
        odom_ang_sigma: odometry angular noise
        meas_rng_noise: measurement linear noise
        meas_ang_noise: measurement angular noise
        '''
        
        # Copy parameters
        self.odom_lin_sigma = odom_lin_sigma
        self.odom_ang_sigma = odom_ang_sigma
        self.meas_rng_noise = meas_rng_noise
        self.meas_ang_noise = meas_ang_noise
        self.chi_thres = .103 # TODO chose your own value
       
        # Odometry uncertainty 
        self.Qk = np.array([[ self.odom_lin_sigma**2, 0, 0],\
                            [ 0, self.odom_lin_sigma**2, 0 ],\
                            [ 0, 0, self.odom_ang_sigma**2]])
        
        # Measurements uncertainty
        self.Rk=np.eye(2)
        self.Rk[0,0] = self.meas_rng_noise
        self.Rk[1,1] = self.meas_ang_noise
        
        # State vector initialization
        self.xk = np.array([x0,y0,theta0]) # Position
        self.Pk = np.zeros((3,3)) # Uncertainty
        
        # Initialize buffer for forcing observing n times a feature before 
        # adding it to the map
        self.featureObservedN = np.array([])
        self.min_observations = 5

        # Matrix to calculate Jacobian
        # Calculated_Jacobian = ...
    #========================================================================
    def get_number_of_features_in_map(self):
        '''
        returns the number of features in the map
        '''
        return (self.xk.size-3)/2
    
    #========================================================================
    def get_polar_line(self, line, odom):
        '''
        Transforms a line from [x1 y1 x2 y2] from the world frame to the
        vehicle frame using odometry [x y ang].
        Returns [range thAutonomous Robotic VehiclesProject•frequency [1.0]:  sets a different frequency for the bagfile to publish the data (if you don’t have apowerful computer you can make it slower).•pause [false]:  if true sets the rosbag play node to pause so the messages can be posted step by stepby pressings.•simulator  [false]:  if  true  launches  gazebo  simulator  with  a  simple  room.   If  this  option  is  used,run the teleoperation node for the turtlebot in a different terminal so you can drive arround theturtlebot.•dataset1 [false]:  if the data is coming from a bagfile, dataset 1 is used when set to true (real datasetwith ground truth map).•dataset2 [false]:  if the data is coming from a bagfile, dataset 2 is used when set to true (real datasetwithout ground truth map).•dataset3 [true]:  if the data is coming from a bagfile, dataset 3 is used when set to true (syntheticdataset with ground truth map).3.2  PredictionIn thepredictfunction implement the equations:̂xBk|k−1=f(̂xBk−1,̂uk−1k)(1)PBk|k−1=FkPBk−1FTk+GkQkGTk(2)Use  the  measurement  uncertainty  of  the  previous  lab  (0.025m  for  the  linear  movement  noise  and  2degrees for the angular).  Once implemented please check that the uncertainty grows without boundariesas shown in theprediciton.mp4video.  Note thatFkangGkare the ones asked in the second point ofthe theoretical work.3.3  State augmentationThis part is totally new from the previous lab.  In this function the state vector grows with the newlyobserved features in order to build the stochastic map.  Use the non associated observations to enlargethe state vector.  Use the equations of the 3rd point of the theoretical work in this step.  Once this stepis implemented but not the following ones, the state vector should add all observed lines as new features,hence, the state vector will keep growing which might make you computer to slow down or crash.  Youcan see an example instateaugmentation.mp4video.3.4  Data associationCompared to the previous lab, now the lines of the map are not saved as initial and ending point, so theequations for computing distance and transform lines between frames have to change.First,  you  need  to  complete  the  functiontfPolarLinein  order  to  return  the  Jacobians  asked  inthe  theoretical  part.   Once  this  function  is  implemented  useta]
        '''
        # Line points
        x1 = line[0]
        y1 = line[1]
        x2 = line[2]
        y2 = line[3]
        
        # Compute line (a, b, c) and range
        line = np.array([y1-y2, x2-x1, x1*y2-x2*y1])
        pt = np.array([odom[0], odom[1], 1])
        dist = np.dot(pt, line) / np.linalg.norm(line[:2])
        
        # Compute angle
        if dist < 0:
            ang = np.arctan2(line[1], line[0])
        else:
            ang = np.arctan2(-line[1], -line[0])
        
        # Return in the vehicle frame
        return np.array([np.abs(dist), angle_wrap(ang - odom[2])])
        
    #========================================================================
    def predict(self, uk):
        
        '''
        Predicts the position of the robot according to the previous position and the odometry measurements. It also updates the uncertainty of the position
        '''
        #TODO: Program this function
        # - Update self.xk and self.Pk using uk and self.Qk
        self.xk = funcs.comp(self.xk, uk)
             
        # Compound robot with odometry
        
        # Compute jacobians of the composition with respect to robot (A_k) 
        # and odometry (W_k)
        A_k = np.array([[1, 0 -1*(np.sin(self.xk[2])*uk[0] + np.cos(self.xk[2])*uk[1])],
                        [0, 1, np.cos(self.xk[2])*uk[0]-np.sin(self.xk[2])*uk[1]],
                        [0, 0, 1]])
        W_k = np.array([[np.cos(self.xk[2]), -np.sin(self.xk[2]), 0],
                        [np.sin(self.xk[2]), np.cos(self.xk[2]), 0],
                        [0, 0, 1]])
        # Prepare the F_k and G_k matrix for the new uncertainty computation
        F_k = np.array(np.zeros((3,3)))
        G_k = np.array(np.zeros((3,3)))
        # Compute uncertainty
        
        # Update the class variables
        self.Pk = np.dot(F_k, (np.dot(self.Pk, np.transpose(F_k))) + np.dot(G_k,(np.dot(self. Qk,np.transpose(G_k)))))
    

    #========================================================================
        
    def data_association(self, lines):
        '''
        Implements ICCN for each feature of the scan.
        # 
        '''
    
        #TODO: Program this function
        # fore each sensed line do:
        #   1- Transform the sensed line to polar
        #   2- for each feature of the map (in the state vector) compute the 
        #      mahalanobis distance
        #   3- Data association
        
        # Init variable
        Innovk_List   = np.zeros((0,0))
        H_k_List      = np.zeros((0,0))
        Rk_List       = np.zeros((0,0))
        idx_not_associated = np.array(range(lines.shape[0]))


        for i in range(0,lines.shape[0]):
            
            minD = 1e9
            minj = -1

            # Transform sensed line to polar form, no transformation needed.
            z = funcs.get_polar_line(lines[i])

            for j in range (0, self.get_number_of_features_in_map()):

                info = lineDist(lines[i], (i,j))


                # Transform the state vector into polar form in world frame
                h = tfPolarLine(Calculated_Jacobian, self.xk[j])
                H = h[1]

                '''
                # Calculate the innovation (difference between features and observation)
                v = z - h

                # S is uncertainty associated with the innovation
                S = np.add((np.matmul(np.matmul(H,self.Pk),H_T)),Rk)

                S_inv = np.linalg.inv(S)

                v_T = np.transpose(v)
                # Compute Mahalanobis Distance 
                D = np.matmul(np.matmul(v_T,S_inv),v)


                # Check if the observed line is the one with smallest
                # mahalanobis distance
                '''
                
                H_T = np.transpose(info[3])
                
                D = info[0]


                if np.sqrt(D) < minD and not islonger:
                    minj = j
                    minz = z
                    minh = info[2]
                    minH = info[3]
                    minv = info[1]
                    minS = S
                    minD = D


                # This is actually supposed to be in the first for loop. 
                # Show example during monday meeting.
                # This also means that minD cannot be passed forward.
                # Minimum distance below threshold
                if minD < self.chi_thres:
                    print("\t{} -> {}".format(minz, minh))
                    # Append results
                    # position?
                    H_k_List.append(minH)
                    Innovk_List.append(minv)
                    Sk_list.append(minS)
                    R_k_list.append(self.Rk)

                else:
                    idx_not_associated.append([i, minj])


        return Innovk_List, H_k_List, Rk_List, idx_not_associated
        
    #========================================================================
    def update_position(self, Innovk_List, H_k_List, Rk_List):
        '''
        Updates the position of the robot according to the given the position
        and the data association parameters.
        Returns state vector and uncertainty.
        
        '''
        #TODO: Program this function
        if Innovk_List.shape[0] < 1:
            return
            
        H_T = n.transpose(H_k_List)
        S = np.matmul((np.matmul(H,self.Pk)),H_T) + self.Rk
        S_inv = np.linalg.inv(S)

        # Kalman Gain
        K = np.dot(self.Pk, np.dot(np.transpose(H_k_List), np.linalg.inv(s)))

        # Update Position
        self.xk = self.xk + np.dot(K, v)

        # Update Uncertainty
        self.Pk = np.dot((np.dot((eyes(n) - np.dot(K, H)), self.Pk)), (np.transpose(np.dot((eyes(n) - np.dot(K, H)), self.Pk)))) + np.dot(K, np.dot(R, np.transpose(K)))


    #========================================================================
    def state_augmentation(self, lines, idx):
        '''
        given the whole set of lines read by the kinect sensor and the
        indexes that have not been associated augment the state vector to 
        introduce the new features
        '''
        # If no features to add to the map exit function
        if idx.size<1:
            return
        
        
        # TODO Program this function
        for i in range(0, lines.shape[0]):
            
            idx_coord = idx[i]

            for j in range(0, self.xk.shape[0]):
                if idx_coord[j] != self.xk[i,:]:
            

    #========================================================================
    def tfPolarLine(self,tf,line):
        '''
        Transforms a polar line in the robot frame to a polar line in the
        world frame
        '''
        # Decompose transformation
        # Decompose transformation
        x_x = tf[0]
        x_y = tf[1]
        x_ang = tf[2]  
        
        # Compute the new phi
        phi = angle_wrap(line[1] + x_ang)
        
        rho_ = line[0] + x_x * np.cos(phi) + x_y * np.sin(phi)
        sign = 1
        if rho_ < 0:
            rho_ = -rho_
            phi = angle_wrap(phi+pi)   
            sign = -1
        
        # Allocate jacobians
        H_tf = np.zeros((2,3))
        H_line = np.eye(2)

        # TODO: Evaluate jacobian respect to transformation
        
        # TODO: Evaluate jacobian respect to line
                
        return np.array([rho,phi]), H_tf, H_line
                
    #========================================================================
    def lineDist(self,z,idx):
       
        '''
        Given a line and an index of the state vector it computes the
        distance between both lines
        '''        
        # TODO program this function
                
        # Transform the map line into robot frame and compute jacobians
        h = tfPolarLine(Calculated_Jacobian, self.xk[j])
        H_position = h[1]
        H_line = h[2]
        
        '''
        Convert line world frame to robot frame bc state vector is in robot frame.
        Use transformed line (world --> robot) to calculate distance form robot to line.
        After distance is found, convert back to world frame and store in the state vector.       


        # Allocate overall jacobian
        H = []
        
        # Concatenate position jacobians and place them into the position
        
        # Place the position of the jacobian with respect to the line in its
        # position
        
        # Calculate innovation
        v = z - h

        H_T = np.transpose(H)
        
        # Calculate innovation uncertainty
        S = np.add((np.matmul(np.matmul(H,self.Pk),H_T)),Rk)

        S_inv = np.linalg.inv(S)

        v_T = np.transpose(v)
  
        # Calculate mahalanobis distance
        D = np.matmul(np.matmul(v_T,S_inv),v)
        
        return D,v,h,H
