import numpy as np
import matplotlib.pyplot as plt

N = 15
x0 = np.array([0, 0])
xf = np.array([N-1, N-1])

W = np.zeros((N, N, 2))

# we define a camera C by its position (x,y), its orientation theta and its angle alpha
cam_x, cam_y = N-1, N-3
cam = np.array([cam_x, cam_y])
# points towards (9, 0)
theta = 5*np.pi/4
alpha = .5

camera_points = []
# for each point x in the grid
for i in range(N):
    for j in range(N):
        # print("ojojojoj")
        # if x is in the scope of the camera, i.e. if the vector x - cam is in the cone defined by the camera
        if (i != cam_x or j != cam_y) and np.dot((np.array([i, j]) - cam)/np.linalg.norm(np.array([i, j]) - cam), np.array([np.cos(theta), np.sin(theta)])) > np.cos(alpha/2):
            camera_points.append(np.array([i, j]))
            W[i, j] = (x0 - np.array([i, j]))/np.linalg.norm(x0 - np.array([i, j]))
# set W for cam
W[cam_x, cam_y] = - cam/np.linalg.norm(cam)# add cam
camera_points.append(cam)
# define o as the set of points in the square defined by points (3, 3), (3, 7), (7, 7), (7, 3)
o = []
# for i in range(3, 8):
#     for j in range(3, 8):
#         o.append(np.array([i, j]))

with open('uarray.npy', 'rb') as f:
    U = np.load(f)

print(U[0:5, 0:5])
# visualize U
print(U[6, 6], U[7, 6], U[6, 7])
print(W[6, 6], W[7, 6], W[6, 7])
plt.imshow(U)
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

print(path)


# visualize path
for i in range(N):
    for j in range(N):
        plt.plot(i, j, 'wo')
for i in range(len(path)):
    plt.plot(path[i][0], path[i][1], 'go')
# visualize obstacle
for point in o:
    plt.plot(point[0], point[1], 'ro')
# visualize camera
for point in camera_points:
    # if point is not in path
    if not any((point == x).all() for x in path):
        plt.plot(point[0], point[1], 'bo')
    else: # plot in purple
        plt.plot(point[0], point[1], 'mo')
# plot point (6, 7) in yellow
# plt.plot(6, 7, 'yo')
# print(U[6, 6], U[6, 7])
plt.show()
