import numpy as np
import matplotlib.pyplot as plt
from upwind import main_U
import time


def path_finder(N, x0, xf, camera_list, obstacle_list):
    # compute U
    print("---------------------")
    print("Computing U...")
    U = main_U(N, x0, xf, camera_list, obstacle_list)
    print("---------------------")

    # compute path from xf to x0
    path = []
    xf = np.array([N-1, N-1])
    path.append(xf)
    while np.linalg.norm(path[-1] - x0) > .5:
        neighbors = [np.array([path[-1][0]-1, path[-1][1]]), np.array([path[-1][0]+1, path[-1][1]]), np.array([path[-1][0], path[-1][1]-1]), np.array([path[-1][0], path[-1][1]+1]), np.array([path[-1][0]-1, path[-1][1]-1]), np.array([path[-1][0]-1, path[-1][1]+1]), np.array([path[-1][0]+1, path[-1][1]-1]), np.array([path[-1][0]+1, path[-1][1]+1])]
        smallest_U = np.inf
        smallest_U_point = np.array([0, 0])
        for neighbor in neighbors:
            if neighbor[0] >= 0 and neighbor[0] < N and neighbor[1] >= 0 and neighbor[1] < N and U[neighbor[0], neighbor[1]] < smallest_U:
                smallest_U = U[neighbor[0], neighbor[1]]
                smallest_U_point = neighbor
        path.append(smallest_U_point)
        # if the current point is the same as the preprevious one (i.e. the path is stuck), then break
        if len(path) > 2 and np.linalg.norm(path[-1] - path[-3]) < .5:
            print("stuck at", path[-1], path[-2])
            break
        # if the current point is the same as the last, break too
        if len(path) > 1 and np.linalg.norm(path[-1] - path[-2]) < .5:
            print("stuck at", path[-1], path[-2])
            break
    print("Path computed.")
    return path

# main
if __name__ == "__main__":
    N = 5
    x0 = np.array([0, 0])
    xf = np.array([N-1, N-1])

    # defining obstacles
        # defining obstacles
    obstacle_list = []
    # for i in range(3, 8):
    #     for j in range(3, 8):
    #         o.append(np.array([i, j]))

    # defining cameras
    camera_list = []
    # we define a camera C by its position (x,y), its orientation theta and its angle alpha
    cam_x, cam_y = 4, 0
    cam = np.array([cam_x, cam_y])
    # points towards (2, 0)
    theta = 4.288
    alpha = .5
    camera_list.append([cam, theta, alpha])

    start_time = time.time()
    path = path_finder(N, x0, xf, obstacle_list, camera_list)
    print("Path: ", path)
    print("--- %s seconds ---" % (time.time() - start_time))