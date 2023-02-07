import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize, rosen, rosen_der
import time

N = 5

doublon = 0

def np_belongs(x, L):
    for i in range(len(L)):
        if np.array_equal(x, L[i]):
            return True
    return False

nfconstant = 1.5
# NF(x) is the set of segments [xj, xk] where [xj, xk] is in AF, and there exists x' on [xj, xk] such that x' is at distance at most nfconstant from x
# initialize empty list
def compute_NF(x, AF):
    NF = []
    for i in range(len(AF)):
        # calculate the distance between x and the segment AF[i]
        # if the distance is smaller than nfconstant, add AF[i] to NF
        if np.linalg.norm(x - AF[i][0]) <= nfconstant or np.linalg.norm(x - AF[i][1]) <= nfconstant:
            # if both points are different and different from x
            if not np.array_equal(AF[i][0], AF[i][1]) and not np.array_equal(AF[i][0], x) and not np.array_equal(AF[i][1], x):
                # print(AF[i].shape)
                NF.append(AF[i])
    return NF


xf = np.array([N-1, N-1])
x0 = np.array([0, 0])
# testing
V_a = 1.
W = np.zeros((N, N, 2))

# define o as the set of points in the square defined by points (3, 3), (3, 7), (7, 7), (7, 3)
o = []
# for i in range(3, 8):
#     for j in range(3, 8):
#         o.append(np.array([i, j]))

# W = np.zeros((N, N, 2))
W = np.zeros((N, N))
# we define a camera C by its position (x,y), its orientation theta and its angle alpha
cam_x, cam_y = 4, 0
cam = np.array([cam_x, cam_y])
# points towards (2, 0)
theta = 2.
alpha = .5

camera_points = []
# for each point x in the grid
for i in range(N):
    for j in range(N):
        # print("ojojojoj")
        # if x is in the scope of the camera, i.e. if the vector x - cam is in the cone defined by the camera
        if (i != cam_x or j != cam_y) and np.dot((np.array([i, j]) - cam)/np.linalg.norm(np.array([i, j]) - cam), np.array([np.cos(theta), np.sin(theta)])) > np.cos(alpha/2):
            print("(i,j) = ", i, j)
            # W[i, j] = 1. * (x0 - np.array([i, j]))/np.linalg.norm(x0 - np.array([i, j]))
            W[i, j] = 1.
# set W for cam
# W[cam_x, cam_y] = - 1. * cam/np.linalg.norm(cam)
W[cam_x, cam_y] = 1.
# plot the camera points with matplotlib
# plt.scatter([x[0] for x in camera_points], [x[1] for x in camera_points])
# # plot point (1, 0) in red
# plt.scatter([1], [0], color='red')
# plt.show()

# # plot W is arrows
# for i in range(N):
#     for j in range(N):
#         plt.plot([i, i+W[i, j, 0]], [j, j+W[i, j, 1]], color='red')
# plt.show()

# plot camera points
for i in range(N):
    for j in range(N):
        if not np.array_equal(W[i, j], 0):
            plt.plot(i, j, 'bo')
        else: # plot in black
            plt.plot(i, j, 'ko')
plt.show()

# f = lambda x, u: np.exp(-np.dot(u-x0, W[int(x[0]), int(x[1])]))
f = lambda x, u: 2. - W[int(x[0]), int(x[1])]
# print(compute_V(np.array([2,2]), AF, V_a, W, f, U))
def exists_considered(M):
    for i in range(N):
        for j in range(N):
            if M[i, j] == 1:
                return True, np.array([i,j])
    return False, 0

def compute_AcceptedFront(M):
    AcceptedFront = []
    # fill AcceptedFront
    for i in range(N):
        for j in range(N):
            # [i, j] is not in o and [i, j] is accepted
            if M[i, j] == 0 and not np_belongs([i, j], o):
                for k in range(-1, 2):
                    for l in range(-1, 2):
                        # if not in o
                        if i+k >= 0 and i+k < N and j+l >= 0 and j+l < N and not np_belongs([i+k, j+l], o):
                            if M[i+k, j+l] == 1:
                                # if already in AcceptedFront
                                if not np_belongs(np.array([i, j]), AcceptedFront):
                                    AcceptedFront.append(np.array([i, j]))
                                break
    return AcceptedFront


def compute_AF(M, AcceptedFront):
    # initialize empty list
    AF = []
    # fill AF
    for i in range(len(AcceptedFront)):
        # print("point try", AcceptedFront[i])
        for k in range(-1, 2):
            for l in range(-1, 2):
                if (k != 0 or l != 0) and AcceptedFront[i][0]+k >= 0 and AcceptedFront[i][0]+k < N and AcceptedFront[i][1]+l >= 0 and AcceptedFront[i][1]+l < N:
                    # if the point is in AcceptedFront
                    # print("try", [AcceptedFront[i][0]+k, AcceptedFront[i][1]+l])
                    if np_belongs([AcceptedFront[i][0]+k, AcceptedFront[i][1]+l], AcceptedFront):
                        # if both points are adjacent to a considered point
                        has_considered_neighbor = False
                        for m in range(-1, 2):
                            for n in range(-1, 2):
                                if AcceptedFront[i][0]+k+m >= 0 and AcceptedFront[i][0]+k+m < N and AcceptedFront[i][1]+l+n >= 0 and AcceptedFront[i][1]+l+n < N:
                                    if M[AcceptedFront[i][0]+k+m, AcceptedFront[i][1]+l+n] == 1:
                                        has_considered_neighbor = True
                                        break
                            if has_considered_neighbor:
                                break
                        has_considered_neighbor2 = False
                        # same for AcceptedFront[i]
                        for m in range(-1, 2):
                            for n in range(-1, 2):
                                if AcceptedFront[i][0]+m >= 0 and AcceptedFront[i][0]+m < N and AcceptedFront[i][1]+n >= 0 and AcceptedFront[i][1]+n < N:
                                    if M[AcceptedFront[i][0]+m, AcceptedFront[i][1]+n] == 1:
                                        has_considered_neighbor2 = True
                                        break
                            if has_considered_neighbor2:
                                break
                        if has_considered_neighbor and has_considered_neighbor2:
                            # if already in AF
                            if not np_belongs([AcceptedFront[i], np.array([AcceptedFront[i][0]+k, AcceptedFront[i][1]+l])], AF):
                                AF.append([AcceptedFront[i], np.array([AcceptedFront[i][0]+k, AcceptedFront[i][1]+l])])
    return AF
history = {}

def compute_U(x, xj, xk, uj, uk):
    # minimize ||theta xj + (1-theta) xk - x||/f(x, (theta xj + (1-theta) xk - x)/ ||theta xj + (1-theta) xk - x||) + theta uj + (1-theta) uk
    # with theta in [0, 1]
    # with scipy.optimize.minimize
    # print("x, xj, xk", x, xj, xk)
    # check if [x, xj, xk, uj, uk] is in history
    if str([x, xj, xk, uj, uk]) in history:
        global doublon
        if doublon == 0:
            print("doublon")
            doublon += 1

        return history[str([x, xj, xk, uj, uk])]
    bnds = ((0, 1),)
    minig = lambda theta: f(x, (theta*xj + (1-theta)*xk - x)/np.linalg.norm(theta*xj + (1-theta)*xk - x))
    g = lambda theta: np.linalg.norm(theta*xj + (1-theta)*xk - x)/f(x, (theta*xj + (1-theta)*xk - x)/np.linalg.norm(theta*xj + (1-theta)*xk - x)) + theta*uj + (1-theta)*uk
    # g = lambda theta: np.linalg.norm(theta*xj + (1-theta)*xk - x) + theta*uj + (1-theta)*uk
    # print("g(0.5)", g(0.5))
    # x_test is 100 random points between 0 and 1
    x_test = np.random.rand(100)
    y_test = []
    for i in range(len(x_test)):
        y_test.append(g(x_test[i]))
        # if minig is zero
        if minig(x_test[i]) == 0.:
            print("error with minig:", x, xj, xk, x_test[i], minig(x_test[i]))
    # theta_min = minimize(g, 0.5, bounds=bnds).x
    # print("theta_min", theta_min)
    # return minimum y_test
    # print(y_test)
    # plot x_test and y_test
    # plt.plot(x_test, y_test)
    # plt.show()
    res = np.min(y_test)
    # print("res: ", res)
    theta_min = x_test[np.argmin(y_test)]
    # redo a search around theta_min
    new_x = np.linspace(theta_min-0.1, theta_min+0.1, 10)
    new_y = []
    for i in range(len(new_x)):
        new_y.append(g(new_x[i]))
    res = np.min(new_y)
    theta_min = new_x[np.argmin(new_y)]
    # print("min point", theta_min*xj + (1-theta_min)*xk)
    # print("u value", res)
    history[str([x, xj, xk, uj, uk])] = res
    return res

def compute_U_naive(x, xj, xk, uj, uk):
    # minimize ||theta xj + (1-theta) xk - x||/f(x, (theta xj + (1-theta) xk - x)/ ||theta xj + (1-theta) xk - x||) + theta uj + (1-theta) uk
    # with theta in [0, 1]
    # with scipy.optimize.minimize
    # print("x, xj, xk", x, xj, xk)
    bnds = ((0, 1),)
    minig = lambda theta: f(x, (theta*xj + (1-theta)*xk - x)/np.linalg.norm(theta*xj + (1-theta)*xk - x))
    g = lambda theta: np.linalg.norm(theta*xj + (1-theta)*xk - x)/f(x, (theta*xj + (1-theta)*xk - x)/np.linalg.norm(theta*xj + (1-theta)*xk - x)) + theta*uj + (1-theta)*uk
    # apply gradient descent on g to find theta_min
    learning_rate = 0.1
    theta = 0.5
    theta_memory1 = theta
    theta_memory2 = theta
    for i in range(50):
        theta_memory1 = theta
        theta_memory2 = theta_memory1
        theta = theta - learning_rate * (g(theta+learning_rate) - g(theta))
        # if we are stuck, i.e. if theta is theta_memory1 or theta_memory2 then we stop
        if theta == theta_memory1 or theta == theta_memory2:
            break
    return g(theta)


# Main Algorithm
# initialize U to infinity and all points at far
print("Starting...")
x0 = np.array([0, 0])
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
# print("exemple", U[0, 1])
U[1, 0] = 1 / f(np.array([1, 0]), - np.array([1, 0]))
# print("exemple", U[1, 0])
# plt.plot([0, 1], [0, 0], 'ro')
# plt.show()

# move all points adjacent to boundary to considered and evaluate U
for i in range(N):
    for j in range(N):
        if M[i, j] == 2:
            for k in range(-1, 2):
                for l in range(-1, 2):
                    if (k != 0 or l != 0) and i+k >= 0 and i+k < N and j+l >= 0 and j+l < N:
                        if M[i+k, j+l] == 0:
                            M[i, j] = 1
                            U[i, j] = compute_U(np.array([i, j]), np.array([i+k, j+l]), np.array([i+k, j+l]), U[i+k, j+l], U[i+k, j+l])
                            break
                if M[i, j] == 1:
                    break

# print(M[0:4, 0:4])

Accepted_Front = compute_AcceptedFront(M)
AF = compute_AF(M, Accepted_Front)
for i in range(N):
    for j in range(N):
        if M[i, j] == 1:
            x_NF = compute_NF(np.array([i, j]), AF)
            for couple in x_NF:
                # if U is not infinity
                if U[couple[0][0], couple[0][1]] != np.inf and U[couple[1][0], couple[1][1]] != np.inf:
                    U[i, j] = np.min([U[i, j], compute_U(np.array([i, j]), couple[0], couple[1], U[couple[0][0], couple[0][1]], U[couple[1][0], couple[1][1]])])


AcceptedFront = compute_AcceptedFront(M)
AF = compute_AF(M, AcceptedFront)


count = 0

def visualize_U(U):
    plt.imshow(U, cmap='hot', interpolation='nearest')
    plt.show()

visualize_U(U)

while exists_considered(M)[0]:
    count += 1
    if count % 10 == 0:
        print("iteration: ", count)
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
    AcceptedFront = compute_AcceptedFront(M)
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
                        U[i, j] = np.min([U[i, j], compute_U(np.array([i, j]), couple[0], couple[1], U[couple[0][0], couple[0][1]], U[couple[1][0], couple[1][1]])])
                        # print("U[i, j]: ", U[i, j])
    # visualize_U(U)



print("\nDone")
# print(U[0:9, 0:9])
# save U
with open('uarray.npy', 'wb') as f:
    np.save(f, U)


# visualize u
plt.imshow(U)
plt.show()

# # compute path from xf to x0
# path = []
# xf = np.array([9, 9])
# path.append(xf)
# while np.linalg.norm(path[-1] - x0) > .5:
#     neighbors = [np.array([path[-1][0]-1, path[-1][1]]), np.array([path[-1][0]+1, path[-1][1]]), np.array([path[-1][0], path[-1][1]-1]), np.array([path[-1][0], path[-1][1]+1]), np.array([path[-1][0]-1, path[-1][1]-1]), np.array([path[-1][0]-1, path[-1][1]+1]), np.array([path[-1][0]+1, path[-1][1]-1]), np.array([path[-1][0]+1, path[-1][1]+1])]
#     smallest_U = np.inf
#     smallest_U_point = np.array([0, 0])
#     for neighbor in neighbors:
#         if neighbor[0] >= 0 and neighbor[0] < N and neighbor[1] >= 0 and neighbor[1] < N and U[neighbor[0], neighbor[1]] < smallest_U:
#             smallest_U = U[neighbor[0], neighbor[1]]
#             smallest_U_point = neighbor
#     path.append(smallest_U_point)

# print(path)


# # visualize path
# for i in range(N):
#     for j in range(N):
#         plt.plot(i, j, 'wo')
# for i in range(len(path)):
#     plt.plot(path[i][0], path[i][1], 'go')
# # visualize obstacle
# for point in o:
#     plt.plot(point[0], point[1], 'ro')
# plt.show()
