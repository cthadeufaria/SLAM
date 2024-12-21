import time
import numpy as np
from slam_front_end import Model
from slam_back_end import Filter



def detect_corners(measurements, delta=0.5):
    angles = np.linspace(-np.pi/6, np.pi/6, 61)
    distances = measurements[:61]
    corners = []

    smoothed_distances = np.convolve(distances, np.ones(3)/3, mode='same')

    for i in range(1, len(smoothed_distances) - 1):
        if (smoothed_distances[i-1] - smoothed_distances[i] > delta) and (smoothed_distances[i+1] - smoothed_distances[i] > delta):
            corners.append((smoothed_distances[i], angles[i]))  # Concave corner

        if (smoothed_distances[i] - smoothed_distances[i-1] > delta) and (smoothed_distances[i] - smoothed_distances[i+1] > delta):
            corners.append((smoothed_distances[i], angles[i]))  # Convex corner

    return corners


def main():
    model = Model()
    ukf = Filter()
    estimated_states = []
    corners = []

    with open("data/data_slam.txt", "r") as file:
        data = file.readlines()

    for row in data:
        measurements = [float(value) for value in row.strip().split()]
        
        corners.append(detect_corners(measurements))
        
        ukf.update(measurements[:2])
        estimated_states.append(ukf.get_state())

        if len(corners[-1]) > 0:
            for corner in corners[-1]:
                model.augment(corner)

        time.sleep(0.5)

    print("end")



if __name__ == "__main__":
    main()