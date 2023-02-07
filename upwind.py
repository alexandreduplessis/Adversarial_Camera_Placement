import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize, rosen, rosen_der
import time
from tqdm import tqdm
from utils import *

np.warnings.filterwarnings('ignore', category=np.VisibleDeprecationWarning)

start_time = time.time()

def main_U(N, x0, xf, camera_list, obstacle_list):
    # nfconstant must be higher than the maximum distance between two points in the grid, so...
    nfconstant = 1.5
    o = obstacle_list

    # W = np.zeros((N, N, 2))
    W = np.zeros((N, N))

    ###############################
    # Cameras setting
    ###############################
    camera_points = []
    # for each point x in the grid
    for i in range(N):
        for j in range(N):
            for cam in camera_list:
                # if x is in the scope of the camera, i.e. if the vector x - cam is in the cone defined by the camera
                if (i != cam[0][0] or j != cam[0][1]) and np.dot((np.array([i, j]) - cam[0])/np.linalg.norm(np.array([i, j]) - cam[0]), np.array([np.cos(cam[1]), np.sin(cam[1])])) > np.cos(cam[2]/2):
                    # print("(i,j) = ", i, j)
                    # W[i, j] = 1. * (x0 - np.array([i, j]))/np.linalg.norm(x0 - np.array([i, j]))
                    W[i, j] += 1.
                    camera_points.append(np.array([i, j]))
    # set W for cam
    for cam in camera_list:
        cam_x = cam[0][0]
        cam_y = cam[0][1]
        # W[cam_x, cam_y] = - 1. * cam/np.linalg.norm(cam)
        W[cam_x, cam_y] += 1.

    # f = lambda x, u: np.exp(-np.dot(u-x0, W[int(x[0]), int(x[1])]))
    f = lambda x, u: .01 * (len(camera_list) + 1. - W[int(x[0]), int(x[1])])
    history = {}


    # Main Algorithm
    # initialize U to infinity and all points at far
    print("Starting...")
    U = np.ones((N, N)) * np.inf
    V = np.ones((N, N)) * np.inf
    M = np.ones((N, N)) * 2

    # move x0 to accepted and u = 0
    U[x0[0], x0[1]] = 0
    V[x0[0], x0[1]] = 0
    M[x0[0], x0[1]] = 0
    # initialize AF to empty
    AF = []

    print("Initialization done.")

    M[0, 0] = 0
    U[0, 0] = 0

    M[0, 1] = 0
    M[1, 0] = 0
    U[0, 1] = 1 / f(np.array([0, 1]), - np.array([0, 1]))
    U[1, 0] = 1 / f(np.array([1, 0]), - np.array([1, 0]))


    # move all points adjacent to boundary to considered and evaluate U
    for i in range(N):
        for j in range(N):
            if M[i, j] == 2:
                for k in range(-1, 2):
                    for l in range(-1, 2):
                        if (k != 0 or l != 0) and i+k >= 0 and i+k < N and j+l >= 0 and j+l < N:
                            if M[i+k, j+l] == 0:
                                M[i, j] = 1
                                U[i, j], history = compute_U(np.array([i, j]), np.array([i+k, j+l]), np.array([i+k, j+l]), U[i+k, j+l], U[i+k, j+l], f, history)
                                break
                    if M[i, j] == 1:
                        break


    Accepted_Front = compute_AcceptedFront(M, o)
    AF = compute_AF(M, Accepted_Front)
    for i in range(N):
        for j in range(N):
            if M[i, j] == 1:
                x_NF = compute_NF(np.array([i, j]), AF)
                for couple in x_NF:
                    # if U is not infinity
                    if U[couple[0][0], couple[0][1]] != np.inf and U[couple[1][0], couple[1][1]] != np.inf:
                        new_u, history = compute_U(np.array([i, j]), couple[0], couple[1], U[couple[0][0], couple[0][1]], U[couple[1][0], couple[1][1]], f, history)
                        U[i, j] = np.min([U[i, j], new_u])


    AcceptedFront = compute_AcceptedFront(M, o)
    AF = compute_AF(M, AcceptedFront)

    for _ in tqdm(range(N*N)):
        # find the considered point with smallest U
        smallest_U = np.inf
        smallest_U_point = np.array([0, 0])
        for i in range(N):
            for j in range(N):
                if M[i, j] == 1 and U[i, j] <= smallest_U and not np_belongs(np.array([i, j]), o):
                    smallest_U = U[i, j]
                    smallest_U_point = np.array([i, j])
        # print("smallest_U_point: ", smallest_U_point)
        # move the point to accepted
        M[smallest_U_point[0], smallest_U_point[1]] = 0
        # update AF
        AcceptedFront = compute_AcceptedFront(M, o)
        AF = compute_AF(M, AcceptedFront)
        # move far points adjacent to the accepted point to considered
        neighbors = [np.array([smallest_U_point[0]-1, smallest_U_point[1]]), np.array([smallest_U_point[0]+1, smallest_U_point[1]]), np.array([smallest_U_point[0], smallest_U_point[1]-1]), np.array([smallest_U_point[0], smallest_U_point[1]+1]), np.array([smallest_U_point[0]-1, smallest_U_point[1]-1]), np.array([smallest_U_point[0]-1, smallest_U_point[1]+1]), np.array([smallest_U_point[0]+1, smallest_U_point[1]-1]), np.array([smallest_U_point[0]+1, smallest_U_point[1]+1])]
        for neighbor in neighbors:
            # if neighbor is in bounds and is far and is not in o
            if neighbor[0] >= 0 and neighbor[0] < N and neighbor[1] >= 0 and neighbor[1] < N and M[neighbor[0], neighbor[1]] == 2 and not np_belongs(neighbor, o):
                M[neighbor[0], neighbor[1]] = 1
        # for all considered points within distance maxdist from x and not in o, recompute U
        for i in range(N):
            for j in range(N):
                if M[i, j] == 1 and not np_belongs(np.array([i, j]), o):
                    x_NF = compute_NF(np.array([i, j]), AF)
                    for couple in x_NF:
                        # if U is not infinity
                        if U[couple[0][0], couple[0][1]] != np.inf and U[couple[1][0], couple[1][1]] != np.inf:
                            new_u, history = compute_U(np.array([i, j]), couple[0], couple[1], U[couple[0][0], couple[0][1]], U[couple[1][0], couple[1][1]], f, history)
                            U[i, j] = np.min([U[i, j], new_u])
    # verify there is no more considered point with exists_considered
    assert not exists_considered(M)[0]
    print("U computed.")

    return U

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
    cam_x, cam_y = 0, N-1
    cam = np.array([cam_x, cam_y])
    # points towards (2, 0)
    theta = -np.pi/4
    alpha = .8
    camera_list.append([cam, theta, alpha])

    start_time = time.time()
    U = main_U(N, x0, xf, obstacle_list, camera_list)
    print("--- %s seconds ---" % (time.time() - start_time))

    # plot U
    plt.imshow(U)
    plt.show()