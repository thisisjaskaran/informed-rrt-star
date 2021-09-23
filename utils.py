import numpy as np

def euclidean_distance(point_1, point_2):
    return np.sqrt( (point_1[0] - point_2[0])**2 + (point_1[1] - point_2[1])**2)