

import numpy as np



# TODO Part 3: Comment the code explaining each part
class kalman_filter:
    
    # TODO Part 3: Initialize the covariances and the states    
    def __init__(self, P,Q,R, x, dt):
        #Initialise the matrices as the passed in values 
        self.P = P
        self.Q = Q
        self.R = R 
        self.x = x
        self.dt = dt
        
    # TODO Part 3: Replace the matrices with Jacobians where needed        
    def predict(self):

        #Updating the state transition matrix and measurement matrix with their respective jacobians 
        #A should be G but we are using A to be consistent with the template 
        self.A = self.jacobian_A() #State transition matrix

        self.C = self.jacobian_H() #Where H is the measurement matrix
        
        #Predicting the mean (state)
        self.motion_model()

        #Predicting the covariance (P matrix/uncertainty)
        self.P= np.dot( np.dot(self.A, self.P), self.A.T) + self.Q

    # TODO Part 3: Replace the matrices with Jacobians where needed
    def update(self, z):
        #Already implemented in given template

        #Calculating the denominator of Kalman Gain
        S=np.dot(np.dot(self.C, self.P), self.C.T) + self.R
        
        #Calcluate the Kalman Gain
        kalman_gain=np.dot(np.dot(self.P, self.C.T), np.linalg.inv(S))
        
        #Calcluating Innovation (difference btw actual and expected measurement) 
        surprise_error= z - self.measurement_model()

        #Calculating the estimated mean/state
        #Done by updating the mean/state based on Kalman Gain and measurements
        self.x=self.x + np.dot(kalman_gain, surprise_error)

        #Calculating the estimated covariance (uncertainty)
        #Done by updating the covariance (uncertainty) based on Kalman Gain and measurements
        self.P=np.dot( (np.eye(self.A.shape[0]) - np.dot(kalman_gain, self.C)) , self.P)
        
    
    # TODO Part 3: Implement here the measurement model
    def measurement_model(self):
        x, y, th, w, v, vdot = self.x
        #Making expected measurement based on predicited mean/state
        #from tutorial: vdot = ax, ay = v*w
        return np.array([
            v, # v
            w,# w
            vdot, # ax
            v * w, # ay
        ])
        
    # TODO Part 3: Impelment the motion model (state-transition matrice)
    def motion_model(self):
        
        x, y, th, w, v, vdot = self.x
        dt = self.dt

        #Computing the predicted mean/state based on previous mean/state and inputs 
        self.x = np.array([
            x + v * np.cos(th) * dt, #x component for v
            y + v * np.sin(th) * dt, #y component for v
            th + w * dt,
            w,
            v  + vdot*dt,
            vdot,
        ])
        


    
    def jacobian_A(self):
        x, y, th, w, v, vdot = self.x
        dt = self.dt
        #Calculating the jacobian btw predicted state and previous state 
        return np.array([
            #x, y,               th, w,             v, vdot
            [1, 0,              -v*np.sin(th)*dt, 0,          np.cos(th)*dt,  0],
            [0, 1,              v*np.cos(th)*dt, 0,          np.sin(th)*dt,  0],
            [0, 0,                1, dt,           0,  0],
            [0, 0,                0, 1,            0,  0],
            [0, 0,                0, 0,            1,  dt],
            [0, 0,                0, 0,            0,  1 ]
        ])
    
    
    # TODO Part 3: Implement here the jacobian of the H matrix (measurements)    
    def jacobian_H(self):
        x, y, th, w, v, vdot=self.x
        #Calculating the jacobian of measurement matrix based on measurement model
        return np.array([
            #x, y,th, w, v,vdot
            [0,0,0  , 0, 1, 0], # v
            [0,0,0  , 1, 0, 0], # w
            [0,0,0  , 0, 0, 1], # ax
            [0,0,0  , v, w, 0], # ay
        ])
        
    # TODO Part 3: return the states here    
    def get_states(self):
        #Returning the estimated state
        return self.x
