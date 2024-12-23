import numpy as np
import matplotlib.pyplot as plt
from slam_ukf import SLAM


def detect_corners(lidar, delta=0.1):
    angles = np.linspace(-np.pi/6, np.pi/6, 61)
    corners = []

    smoothed_distances = np.convolve(lidar, np.ones(3)/3, mode='same')

    for i in range(1, len(smoothed_distances) - 1):
        if (smoothed_distances[i-1] - smoothed_distances[i] > delta) and (smoothed_distances[i+1] - smoothed_distances[i] > delta):
            corners.append((smoothed_distances[i], angles[i]))

        if (smoothed_distances[i] - smoothed_distances[i-1] > delta) and (smoothed_distances[i] - smoothed_distances[i+1] > delta):
            corners.append((smoothed_distances[i], angles[i]))

    return corners


def local_to_global(corner, robot_pose):
    """Convert polar coordinates to global coordinates."""
    r, phi = corner # r, phi (radians)
    x_r, y_r, theta_r = robot_pose # x, y, theta (radians or degrees?)
    x_l = x_r + r * np.cos(phi + theta_r)
    y_l = y_r + r * np.sin(phi + theta_r)
    return x_l, y_l


def global_to_local(corner_global, robot_pose):
    """Convert global coordinates to local coordinates."""
    x_l, y_l = corner_global
    x_r, y_r, theta_r = robot_pose
    dx = x_l - x_r
    dy = y_l - y_r
    r = np.sqrt(dx**2 + dy**2)
    phi = np.arctan2(dy, dx) - theta_r
    return r, phi


def is_new_landmark(corner_global, landmarks, threshold=0.5):
    """Check if the detected corner is a new landmark."""
    index = 0
    x_l, y_l = corner_global
    for landmark in landmarks:
        dx = x_l - landmark[0]
        dy = y_l - landmark[1]
        distance = np.sqrt(dx**2 + dy**2)
        if distance < threshold:
            return False, index
        index += 1
    return True, -1


def main():
    slam = SLAM()
    estimated_states = []

    fig, ax = plt.subplots(figsize=(32, 18))
    ax.set_xlim(-5, 15)
    ax.set_ylim(-10, 5)
    ax.set_xlabel('X coordinate')
    ax.set_ylabel('Y coordinate')
    ax.set_title('SLAM: Robot Trajectory')
    ax.grid(True)
    
    trajectory_line, = ax.plot([], [], 'b-', label="Robot Trajectory")
    scatter_points = ax.scatter([], [], c='red', marker='x', label="Landmarks")
    ax.legend()

    with open("data/data_slam.txt", "r") as file:
        data = file.readlines()

    for row in data:
        measurements = [float(value) for value in row.strip().split()]
        odometry = measurements[:3]
        odometry[2] = np.radians(odometry[2])
        lidar = measurements[3:]

        slam.ukf.predict(odometry=odometry)
        robot_pose = slam.get_state()[:3] # x, y, theta (radians or degrees?)

        detected_corners = detect_corners(lidar)
        
        z = []
        for landmark in slam.landmarks:
            r, phi = global_to_local(landmark, robot_pose)
            z.extend([r, phi])

        if detected_corners:
            for corner in detected_corners: # r, phi
                corner_global = local_to_global(corner, robot_pose)
                if is_new_landmark(corner_global, slam.landmarks)[0]:
                    slam.augment_landmarks(corner_global)
                    z.extend((corner[0], corner[1]))
                else:
                    z[
                        is_new_landmark(corner_global, slam.landmarks)[1]
                    ] = corner[0]
                    z[
                        is_new_landmark(corner_global, slam.landmarks)[1] + 1
                    ] = corner[1]
                        
            slam.ukf.update(z)
            
        estimated_states.append(slam.get_state())

        x_robot = [state[0] for state in estimated_states]
        y_robot = [state[1] for state in estimated_states]
        x_landmarks = [landmark[0] for landmark in slam.landmarks]
        y_landmarks = [landmark[1] for landmark in slam.landmarks]

        trajectory_line.set_data(x_robot, y_robot)
        scatter_points.set_offsets(np.c_[x_landmarks, y_landmarks])

        plt.pause(0.1)
        plt.draw()

    plt.savefig("trajectory_plot.png")

    x_landmarks = [landmark[0] for landmark in slam.landmarks]
    y_landmarks = [landmark[1] for landmark in slam.landmarks]

    plt.figure()
    plt.scatter(x_landmarks, y_landmarks, c='red', marker='x')
    plt.xlabel('X coordinate')
    plt.ylabel('Y coordinate')
    plt.title('Landmarks')
    plt.grid(True)
    plt.savefig("landmarks_plot.png")
    plt.show()

    print("Landmarks:", slam.landmarks)
    local_landmarks = []
    for landmark in slam.landmarks:
        local_landmarks.append(global_to_local(landmark, robot_pose))
    print("Local landmarks:", local_landmarks)



if __name__ == "__main__":
    main()