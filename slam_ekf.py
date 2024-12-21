import numpy as np
from filterpy.kalman import UnscentedKalmanFilter as EKF
from filterpy.kalman import MerweScaledSigmaPoints



class SLAM:
    def __init__(self):
        # Initial pose (x, y, theta)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Initialize landmarks (none initially)
        self.landmarks = []

        # Initial state vector: [x, y, theta]
        self.state = np.array([self.x, self.y, self.theta])

        # Initial process noise
        self.Q = np.diag([0.1, 0.1, np.radians(1)])**2  # Base process noise for x, y, theta

        # Initialize UKF
        self.ukf = EKF(
            dim_x=3, dim_z=2, dt=1, hx=self.hx, fx=self.fx, 
            points=MerweScaledSigmaPoints(n=3, alpha=0.1, beta=2.0, kappa=0.0)
        )  # No measurements initially
        self.ukf.x = self.state
        self.ukf.P = np.eye(3) * 0.2
        self.ukf.Q = self.Q

        self.augmented = False

    # def fx(self, x, dt):
    #     return x
    
    def fx(self, x, dt):
        """
        State transition function for SLAM:
        Updates the robot pose while keeping landmarks stationary in the global frame.
        """
        # Extract robot pose and motion inputs
        delta_x, delta_y, delta_theta = x[:3]
        x_robot, y_robot, theta_robot = self.ukf.x[:3]
        
        # Update robot pose
        x_new = x_robot + delta_x * np.cos(theta_robot)
        y_new = y_robot + delta_x * np.sin(theta_robot)
        theta_new = theta_robot + delta_theta

        # Ensure theta stays within [-π, π] for consistency
        # theta_new = np.arctan2(np.sin(theta_new), np.cos(theta_new))

        # Construct the updated state vector
        pose = np.array([x_new, y_new, theta_new])
        landmarks_flat = np.array(self.landmarks).flatten()
        updated_state = np.hstack([pose, landmarks_flat])
        
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
        self.augmented = True

        self.landmarks.append(new_landmark)

        # Augment the state vector with the new landmark
        self.state = np.hstack([self.state, new_landmark])

        # Increase UKF dimensions
        dim_x = len(self.state)  # Update state dimension
        dim_z = len(self.landmarks) * 2  # Each landmark contributes (range, bearing)

        # Expand the process covariance matrix
        P_new = np.eye(dim_x) * 0.2
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

    def predict_and_update(self, z):
        """Predict and update UKF with measurements."""
        self.ukf.predict()
        if self.augmented:
            self.ukf.update(z)
            self.augmented = False

    def get_state(self):
        """Return the current state estimate."""
        return self.ukf.x