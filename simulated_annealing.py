import numpy as np
import matplotlib.pyplot as plt
import argparse
from path_finder import path_finder
from utils import cost_path
from tqdm import tqdm
from visualization import visualization

def camera_random_setting(N, cam_angle=.5):
    # initialize camera position on the boundary
    # choose one of the 4 boundaries
    boundary = np.random.randint(0, 4)
    # choose a random position on the boundary
    cam_dist = np.random.randint(0, N)
    # initialize camera position
    if boundary == 0:
        cam_x, cam_y = 0, cam_dist
    elif boundary == 1:
        cam_x, cam_y = N-1, cam_dist
    elif boundary == 2:
        cam_x, cam_y = cam_dist, 0
    elif boundary == 3:
        cam_x, cam_y = cam_dist, N-1
    cam = np.array([cam_x, cam_y])
    # initialize camera orientation such that it sees at least one point of the grid
    if boundary == 0:
        theta = np.random.uniform(0, np.pi/2)
    elif boundary == 1:
        theta = np.random.uniform(3*np.pi/2, 2*np.pi)
    elif boundary == 2:
        theta = np.random.uniform(np.pi/2, np.pi)
    elif boundary == 3:
        theta = np.random.uniform(np.pi, 3*np.pi/2)
    
    return [cam, theta, cam_angle]


def main_SA(N, x0, xf, nb_camera, obstacle_list, T, T_min, alpha, max_iter):
    # initialize path
    path = path_finder(N, x0, xf, [], obstacle_list)
    # initialize camera list
    camera_list = []
    for _ in range(nb_camera):
        new_cam = camera_random_setting(N)
        camera_list.append(new_cam)

    # initialize path with camera
    path = path_finder(N, x0, xf, camera_list, obstacle_list)
    # initialize best path
    best_path = path
    # initialize best camera list
    best_camera_list = camera_list
    # initialize best path length
    best_path_length = cost_path(best_path)
    # initialize iteration number
    iter = 0
    # while temperature is above minimum and iteration number is below maximum
    for _ in tqdm(range(max_iter)):
        if T <= T_min:
            break
        # for each camera (could also choose randomly one camera each time)
        for i in range(nb_camera):
            # initialize new camera
            new_cam = camera_random_setting(N)
            # add new camera to list
            new_camera_list = camera_list
            new_camera_list[i] = new_cam
            # initialize new path with new camera
            new_path = path_finder(N, x0, xf, new_camera_list, obstacle_list)
            # initialize new path length
            new_path_length = cost_path(new_path)
            # if new path is better than old path
            if new_path_length > best_path_length:
                # update best path
                best_path = new_path
                # update best camera list
                best_camera_list = new_camera_list
                # update best path length
                best_path_length = new_path_length
            # else if new path is worse than old path
            else:
                # compute probability of accepting new path
                p = np.exp((new_path_length - best_path_length)/T)
                # if new path is accepted
                if np.random.uniform(0, 1) < p:
                    # update best path
                    best_path = new_path
                    # update best camera list
                    best_camera_list = new_camera_list
                    # update best path length
                    best_path_length = new_path_length
        # update temperature
        T = alpha*T
        # update iteration number
        iter += 1
    
    return best_path, best_camera_list

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Parsing command line arguments.')
    parser.add_argument("--visualize", type=int, default=0)
    parser.add_argument("--nbcameras", type=int, default=1)

    args = parser.parse_args()
    visualize = args.visualize
    nb_camera = args.nbcameras

    N = 5
    x0 = np.array([0, 0])
    xf = np.array([N-1, N-1])
    obstacle_list = []

    T = 100
    T_min = 1e-3
    alpha = 0.99
    max_iter = 1
    
    path, camera_list = main_SA(N, x0, xf, nb_camera, obstacle_list, T, T_min, alpha, max_iter)

    print("Camera: ", camera_list)

    if visualize:
        # convert from grid (0, N-1) to real (-2, 2)
        camera_list_real = [[np.array([camera[0][0]*4/(N-1)-2., camera[0][1]*4/(N-1)-2.]), camera[1], camera[2]] for camera in camera_list]
        visualization(N, camera_list_real, obstacle_list)