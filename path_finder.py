import numpy as np
import matplotlib.pyplot as plt
from upwind import main_U
from utils import *
import time
import argparse


def path_finder(N, x0, xf, camera_list, obstacle_list, saveU=0):
    # compute U
    print("---------------------")
    print("Computing U...")
    U = main_U(N, x0, xf, camera_list, obstacle_list)
    print("---------------------")
    
    if saveU:
        image = plt.imshow(U)
        plt.colorbar(image)
        plt.title("U matrix in a {}x{} grid".format(N, N))
        plt.savefig("outputs/uarray.png", format="png")
        with open('outputs/uarray.npy', 'wb') as f:
            np.save(f, U)
        plt.show()
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
    parser = argparse.ArgumentParser(description='Parsing command line arguments.')
    parser.add_argument("--visualize", type=int, default=1)
    parser.add_argument("--saveU", type=int, default=1)
    parser.add_argument("--N", type=int, default=10)

    args = parser.parse_args()
    visualize = args.visualize
    saveU = args.saveU
    N = args.N

    x0 = np.array([0, 0])
    xf = np.array([N-1, N-1])

    # defining obstacles
        # defining obstacles
    obstacle_list = []
    
    # define a square obstacle
    obstacle_list.append([N//2 - 3, N//2 - 3, 6])

    # defining cameras
    # camera_list = []
    # # we define a camera C by its position (x,y), its orientation theta and its angle alpha
    # cam_x, cam_y = 0, N-1
    # cam = np.array([cam_x, cam_y])
    # # points towards (2, 0)
    # theta = -np.pi/4
    # alpha = .8
    # camera_list.append([cam, theta, alpha])
    # camera_list = [[np.array([0, N-1]), -np.pi/4, .5]]
    camera_list = [[np.array([0, N-1]), -np.pi/4, 0.5]]

    start_time = time.time()
    path = path_finder(N, x0, xf, camera_list,  obstacle_list, saveU=saveU)
    print("Path: ", path)
    # compute path length with cost_path
    print("Path length: ", cost_path(path))
    print("--- %s seconds ---" % (time.time() - start_time))

    if visualize:
        camera_points = []
        for cam in camera_list:
            # compute visible points
            visible_points = compute_camera_visible_points(cam, obstacle_list, N)
            for point in visible_points:
                camera_points.append(np.array(point))
        
        for camera in camera_list:
            cam, theta, alpha = camera
            camera_points.append(cam)

        for i in range(N):
            for j in range(N):
                plt.plot(i, j, 'wo')
        for i in range(len(path)):
            plt.plot(path[i][0], path[i][1], 'go')
        # visualize obstacle
        for point in obstacles_list_to_points(obstacle_list):
            plt.plot(point[0], point[1], 'ro')
        # visualize camera
        for point in camera_points:
            # if point is not in path
            if not any((point == x).all() for x in path):
                plt.plot(point[0], point[1], 'bo')
            else: # plot in purple
                plt.plot(point[0], point[1], 'mo')
        plt.savefig("outputs/path_raw.png", format="png")
        plt.title("Optimal path on a {}x{} grid".format(N, N))
        plt.show()