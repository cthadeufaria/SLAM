import numpy as np
from filterpy.kalman import UnscentedKalmanFilter as UKF
from filterpy.kalman import MerweScaledSigmaPoints



class SLAM:
    def __init__(self):
        self.landmarks = []
        self.state = np.array([0., 0., 0.])
        self.Q = np.diag([0.1, 0.1, np.radians(1)])**2  # Base process noise for x, y, theta

        self.ukf = UKF(
            dim_x=3, dim_z=2, dt=1, hx=self.hx, fx=self.fx, 
            points=MerweScaledSigmaPoints(n=3, alpha=0.1, beta=2.0, kappa=0.0)
        )
        self.ukf.x = self.state
        self.ukf.P = np.eye(3) * 0.2
        self.ukf.Q = self.Q

    
    def fx(self, x, dt, odometry):
        """
        State transition function for SLAM:
        Updates the robot pose while keeping landmarks stationary in the global frame.
        """
        delta_x, delta_y, delta_theta = odometry
        x_robot, y_robot, theta_robot = x[:3]
        
        x_new = x_robot + delta_x * np.cos(theta_robot) - delta_y * np.sin(theta_robot)
        y_new = y_robot + delta_x * np.sin(theta_robot) + delta_y * np.cos(theta_robot)
        theta_new = theta_robot + delta_theta

        # theta_new = np.arctan2(np.sin(theta_new), np.cos(theta_new))

        pose = np.array([x_new, y_new, theta_new])
        landmarks_flat = np.array(self.landmarks).flatten()
        updated_state = np.hstack([pose, landmarks_flat])

        print("updated state", updated_state)
        
        return updated_state
    

    def hx(self, x):
        """Measurement model: Calculate expected ranges and bearings to landmarks."""
        ranges = []
        angles = []
        for landmark in self.landmarks:
            dx = landmark[0] - x[0]
            dy = landmark[1] - x[1]
            ranges.append(np.sqrt(dx**2 + dy**2))
            angles.append(np.arctan2(dy, dx) - x[2])
        
        return np.hstack([ranges, angles])


    def augment_landmarks(self, new_landmark):
        """Add a new landmark and augment the state and covariance matrices."""
        self.landmarks.append(new_landmark)

        # Augment the state vector with the new landmark
        self.state = np.hstack([self.state, new_landmark])

        # Increase UKF dimensions
        dim_x = len(self.state)  # Update state dimension
        dim_z = len(self.landmarks) * 2  # Each landmark contributes (range, bearing)

        # Expand the process covariance matrix
        P_new = np.eye(len(self.landmarks)*2 + 3) * 0.2
        P_new[:self.ukf.P.shape[0], :self.ukf.P.shape[1]] = self.ukf.P  # Preserve old covariance

        # Expand the process noise matrix
        Q_new = np.eye(dim_x) * 0.1
        Q_new[:self.Q.shape[0], :self.Q.shape[1]] = self.Q  # Preserve old process noise

        # Reinitialize the UKF with updated dimensions
        self.ukf = UKF(
            dim_x=dim_x,
            dim_z=dim_z,
            dt=1,
            hx=self.hx,
            fx=self.fx,
            points=MerweScaledSigmaPoints(n=dim_x, alpha=0.1, beta=2.0, kappa=0.0),
        )
        self.ukf.x = self.state
        self.ukf.P = P_new
        self.ukf.Q = Q_new


    def get_state(self):
        """Return the current state estimate."""
        return self.ukf.x
