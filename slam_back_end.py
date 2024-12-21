# Inference and estimation of the robot's pose and the map of the environment
import numpy as np
from filterpy.kalman import UnscentedKalmanFilter as UKF
from filterpy.kalman import MerweScaledSigmaPoints



class Filter:
    def __init__(self):
        self.points = MerweScaledSigmaPoints(n=4, alpha=0.1, beta=2.0, kappa=0.0)
        self.ukf = UKF(dim_x=4, dim_z=2, dt=0.5, hx=self.hx, fx=self.fx, points=self.points)
        self.ukf.x = np.array([0., 0., 0., 0.])
        self.ukf.P *= 0.2
        self.ukf.R = np.diag([0.09, np.radians(10)])**2
        self.ukf.Q = np.diag([0.1, 0.1, np.radians(1), 0.1])**2

    def hx(self, x):
        return x[[0, 1]]

    def fx(self, x, dt):
        return x

    def update(self, z):
        self.ukf.predict()
        self.ukf.update(z)

    def get_state(self):
        return self.ukf.x
