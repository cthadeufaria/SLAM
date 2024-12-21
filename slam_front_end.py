import numpy as np

class Model:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.theta = 0
        self.landmarks = []  # List to hold landmark positions
        self.Q = np.diag([0.1, 0.1, np.radians(1)])  # Process noise covariance

        # Initial state vector
        self.Xk = np.array([[self.x], [self.y], [self.theta]])

    def motion_model(self, delta_x, delta_y, delta_theta):
        """
        Predict the next state based on the motion model.
        """
        # Transform odometry from the robot's local frame to the global frame
        x_pred = self.x + delta_x * np.cos(self.theta) - delta_y * np.sin(self.theta)
        y_pred = self.y + delta_x * np.sin(self.theta) + delta_y * np.cos(self.theta)
        theta_pred = self.theta + delta_theta

        return np.array([x_pred, y_pred, theta_pred])

    def update(self, delta_x, delta_y, delta_theta):
        """
        Apply the motion model to update the robot's pose.
        """
        # Predict the next state using the motion model
        predicted_state = self.motion_model(delta_x, delta_y, delta_theta)

        # Update the internal state variables
        self.x, self.y, self.theta = predicted_state

        # Update the state vector
        self.Xk = np.array([[self.x], [self.y], [self.theta]] + self.landmarks)

    def augment(self, landmark):
        # Add new landmark to the state vector
        self.landmarks.append(landmark)
        self.Xk = np.array([[self.x], [self.y], [self.theta]] + self.landmarks)

    def __str__(self):
        return f"x: {self.x}, y: {self.y}, theta: {self.theta}, landmarks: {self.landmarks}"
