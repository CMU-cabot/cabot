# Copyright (c) 2021  IBM Corporation
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import numpy as np
from scipy.linalg import block_diag
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise

def init_kf(initial_state, dt, Q_std, R_std):
    """
    Args:
        initial_state : [x, vx, y, vy, w, h]
        dt : time step to update
        Q_std : Standard deviation to use for process noise covariance matrix
        R_std : Standard deviation to use for measurement noise covariance matrix
    Returns:
        kf: KalmanFilter instance
    """
    kf = KalmanFilter(dim_x=6, dim_z=4)
    
    # state mean and covariance
    kf.x = np.array([initial_state]).T
    kf.P = np.eye(kf.dim_x) * 500.
    
    # no control inputs
    kf.u = 0.
    
    # state transition matrix
    kf.F = np.eye(kf.dim_x)
    kf.F[0, 1] = kf.F[2, 3] = dt
    
    # measurement matrix - maps from state space to observation space, so
    # shape is dim_z x dim_x. Set coefficients for x,y,w,h to 1.0.
    kf.H = np.zeros([kf.dim_z, kf.dim_x])
    kf.H[0, 0] = kf.H[1, 2] = kf.H[2, 4] = kf.H[3, 5] = 1.0
    
    # measurement noise covariance
    kf.R = np.eye(kf.dim_z) * R_std**2
    
    # process noise covariance for x-vx or y-vy pairs
    q = Q_discrete_white_noise(dim=2, dt=dt, var=Q_std**2)
    
    # assume width and height are uncorrelated
    q_wh = np.diag([Q_std**2, Q_std**2])
    
    kf.Q = block_diag(q, q, q_wh)
    
    return {"kf":kf, "missed":0}

def init_kf_fixed_size(initial_state, dt, Q_std, R_std):
    """
    Args:
        initial_state : [x, vx, y, vy]
        dt : time step to update
        Q_std : Standard deviation to use for process noise covariance matrix
        R_std : Standard deviation to use for measurement noise covariance matrix
    Returns:
        kf: KalmanFilter instance
    """
    kf = KalmanFilter(dim_x=4, dim_z=2)
    
    # state mean and covariance
    kf.x = np.array([initial_state]).T
    kf.P = np.eye(kf.dim_x) * 500.
    
    # no control inputs
    kf.u = 0.
    
    # state transition matrix
    kf.F = np.eye(kf.dim_x)
    kf.F[0, 1] = kf.F[2, 3] = dt
    
    # measurement matrix - maps from state space to observation space, so
    # shape is dim_z x dim_x. Set coefficients for x,y to 1.0.
    kf.H = np.zeros([kf.dim_z, kf.dim_x])
    kf.H[0, 0] = kf.H[1, 2] = 1.0
    
    # measurement noise covariance
    kf.R = np.eye(kf.dim_z) * R_std**2
    
    # process noise covariance for x-vx or y-vy pairs
    q = Q_discrete_white_noise(dim=2, dt=dt, var=Q_std**2)
    
    kf.Q = block_diag(q, q)
    
    return {"kf":kf, "missed":0}
