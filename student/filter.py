# ---------------------------------------------------------------------
# Project "Track 3D-Objects Over Time"
# Copyright (C) 2020, Dr. Antje Muntzinger / Dr. Andreas Haja.
#
# Purpose of this file : Kalman filter class
#
# You should have received a copy of the Udacity license together with this program.
#
# https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013
# ----------------------------------------------------------------------
#

# imports
import numpy as np

# add project directory to python path to enable relative imports
import os
import sys
PACKAGE_PARENT = '..'
SCRIPT_DIR = os.path.dirname(os.path.realpath(os.path.join(os.getcwd(), os.path.expanduser(__file__))))
sys.path.append(os.path.normpath(os.path.join(SCRIPT_DIR, PACKAGE_PARENT)))
import misc.params as params 

class Filter:
    '''Kalman filter class'''
    def __init__(self, dt=params.dt, q=params.q):
        self.dt = dt
        self.q = q

    def F(self):
        """
        f is the state transition function state transition function (in the linear case, it is a matrix
        F, also called system matrix system matrix). It tells us how to get from one timestamp to the next:
        x[k] = f(x[k-1])+v = Fx[k-1] + v

        1d case pos + v with t=1
        return np.matrix([[1, 1],
                        [0, 1]])
        2d pos + v
        return np.matrix([[1, 0, dt, 0],
                          [0, 1, 0, dt],
                          [0, 0, 1, 0],
                          [0, 0, 0, 1]])
        """
        ############
        # TODO Step 1: implement and return system matrix F
        ############
        # 3d x + v
        dt = self.dt
        return np.matrix([[1, 0, 0, dt,  0,  0],
                          [0, 1, 0,  0, dt,  0],
                          [0, 0, 1,  0,  0, dt],
                          [0, 0, 0,  1,  0,  0],
                          [0, 0, 0,  0,  1,  0],
                          [0, 0, 0,  0,  0,  1]])

        
        ############
        # END student code
        ############ 

    def Q(self):
        """
        ν∼N(0,Q) is the zero-mean process noise process noise with covariance Q

        1d case:
        return np.matrix([[0, 0],
                          [0, 0]])

        2d case:
        return np.matrix([[q1, 0, q2, 0],
                          [0, q1, 0, q2],
                          [q2, 0, q3, 0],
                          [0, q2, 0,  q3]])

        Here q is a design parameter and should be chosen depending on the expected maximum change in velocity.
        For highly dynamic maneuvers, we could use a higher process noise, e.g. 8 m/s2 would fit for emergency
        braking, whereas for normal situations on a highway e.g. 3 m/s2 (==params.q) could be an adequate choice.
        """
        ############
        # TODO Step 1: implement and return process noise covariance Q
        ############
        q = self.dt * self.q
        q2 = ((self.dt**2)/2) * self.q
        q3 = ((self.dt**3)/3) * self.q

        return np.matrix([[q3,  0,  0, q2,  0,  0],
                          [ 0, q3,  0,  0, q2,  0],
                          [ 0,  0, q3,  0,  0, q2],
                          [q2,  0,  0,  q,  0,  0],
                          [ 0, q2,  0,  0,  q,  0],
                          [ 0,  0, q2,  0,  0,  q]])
        
        ############
        # END student code
        ############ 


    def predict(self, track):
        ############
        # TODO Step 1: predict state x and estimation error covariance P to next timestep, save x and P in track
        ############

        # Implement the F() and Q() functions to calculate a system matrix for constant velocity process model in 3D and the corresponding process noise covariance depending on the current timestep dt.
        F = self.F()
        x = F*track.x # predict the state
        P = F*track.P*F.transpose() + self.Q() # predict the covariance

        #  At the end of the prediction step, save the resulting x and P by calling the functions set_x() and set_P() that are already implemented in student/trackmanagement.py.
        # x is the  state vector. It contains information about position and velocity of the object that you are tracking: (px, py, pz, vx, vy, vz)
        # (ego coordinate system: x forward, y right, z up)
        track.set_x(x)
        # P is the estimation error covariance matrix estimation error covariance matrix, which contains information about
        # the uncertainty of the object's position and velocity.
        track.set_P(P)
        
        ############
        # END student code
        ############ 

    def update(self, track, meas):
        ############
        # TODO Step 1: update state x and covariance P with associated measurement, save x and P in track
        ############
        H = meas.sensor.get_H(track.x)  # measurement matrix
        K = track.P*H.transpose()*np.linalg.inv(self.S(track, meas, H)) # Kalman gain
        x = track.x + K * self.gamma(track, meas) # state update
        P = (np.identity(params.dim_state) - K*H) * track.P # covariance update
        track.set_x(x)
        track.set_P(P)

        ############
        # END student code
        ############ 
        track.update_attributes(meas)
    
    def gamma(self, track, meas):
        ############
        # TODO Step 1: calculate and return residual gamma
        ############
        return meas.z - meas.sensor.get_hx(track.x)
        
        ############
        # END student code
        ############ 

    def S(self, track, meas, H):
        ############
        # TODO Step 1: calculate and return covariance of residual S
        ############
        return (H * track.P * H.transpose()) + meas.R # covirance of residual
        
        ############
        # END student code
        ############ 