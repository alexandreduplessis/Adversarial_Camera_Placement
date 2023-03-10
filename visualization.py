from matplotlib.patches import Polygon, Circle, Rectangle
import numpy as np
import matplotlib.pyplot as plt
from path_finder import path_finder
from utils import *

# Code adapted from a code snippet by Yann Dubois de Mont-Martin

def visualization(N, camera_list, obstacle_list, pos_start=np.array([-2., -2.]), pos_end=np.array([2., 2.])):
    fig, ax = plt.subplots(1, 1, figsize=(2.5, 2.5))
    x_min, y_min, x_max, y_max = -2., -2., 2., 2.

    max_length = np.sqrt((x_max - x_min)**2 + (y_max - y_min)**2)

    # ax setup
    ax.axis([x_min, x_max, y_min, y_max])
    ax.set_aspect("equal")
    ax.set_xticks([])
    ax.set_xticks([], minor=True)
    ax.set_yticks([])
    ax.set_yticks([], minor=True)

    for i in range(len(camera_list)):
        cam_pos = camera_list[i][0]
        cam_orient = camera_list[i][1]
        cam_angle = camera_list[i][2]/2

        center_base = cam_pos
        direc = np.array([np.cos(cam_orient), np.sin(cam_orient)])
        ortho = np.array([-np.sin(cam_orient), np.cos(cam_orient)])

        vertices = np.stack(
            [cam_pos,
            cam_pos + max_length * (direc + np.sin(cam_angle) * ortho),
            cam_pos + max_length * (direc - np.sin(cam_angle) * ortho)],
            axis=0
        )

        cam_cone_plt = ax.add_patch(Polygon(
            vertices,
            fill=True,
            alpha=.5,
            color=(.7, .7, 0.),
            zorder=3,
            lw=0
        ))

    # example obs
    # obs_pos = np.array([0.8, 0.1])
    # obs_radius = 0.2

    # obs_1_plt = ax.add_patch(Circle(
    #     obs_pos,
    #     obs_radius,
    #     fill=True,
    #     alpha=.5,
    #     color=(.3, .1, .3),
    #     zorder=1,
    #     lw=0
    # ))

    # plot square obstacles
    for obs in obstacle_list:
        # obs is in the form (x, y, c) where (x, y) is the position of the lower left corner and c is the side length
        obs_point = np.array([obs[0], obs[1]])
        width, height = obs[2], obs[3]
        obs_1_plt = ax.add_patch(Rectangle(obs_point, width, height))

    # example traj
    xls = np.linspace(x_min, x_max, N)
    yls = np.linspace(y_min, y_max, N)

    X, Y = np.meshgrid(xls, yls)


    # should come extract from HJ on grid
    # be carefull to use center of cells

    # compute x0 and xf, making transition from real to grid
    x0 = (pos_start - np.array([x_min, y_min])) / (x_max - x_min) * (N - 1)
    xf = (pos_end - np.array([x_min, y_min])) / (x_max - x_min) * (N - 1)
    # convert to int values
    x0 = x0.astype(int)
    xf = xf.astype(int)

    # defining obstacles
    grid_obstacle_list = []
    # convert obstacle_list with real values between -2 and 2 to grid values between 0 and N-1
    for obs in obstacle_list:
        obs_radius_1 = obs[2]
        obs_radius_2 = obs[3]
        # convert obs_pos to grid
        obs_x = (obs[0] - x_min) / (x_max - x_min) * (N - 1)
        obs_y = (obs[1] - y_min) / (y_max - y_min) * (N - 1)
        obs_rad_grid_1 = obs_radius_1 / (x_max - x_min) * (N - 1)
        obs_rad_grid_2 = obs_radius_2 / (y_max - y_min) * (N - 1)
        # convert to int
        obs_x = int(obs_x)
        obs_y = int(obs_y)
        obs_radius_1 = int(obs_rad_grid_1)
        obs_radius_2 = int(obs_rad_grid_2)
        grid_obstacle_list.append([obs_x, obs_y, obs_radius_1, obs_radius_2])

    # defining cameras
    camera_list_grid = []
    for camera in camera_list:
        cam_pos = camera[0]
        cam_orient = camera[1]
        cam_angle = camera[2]
        # convert cam_pos to grid
        cam_x = (cam_pos[0] - x_min) / (x_max - x_min) * (N - 1)
        cam_y = (cam_pos[1] - y_min) / (y_max - y_min) * (N - 1)
        # convert to int
        cam_x = int(cam_x)
        cam_y = int(cam_y)
        cam_ind = np.array([cam_x, cam_y])
        camera_list_grid.append([cam_ind, cam_orient, cam_angle])
    path = path_finder(N, x0, xf, camera_list_grid, grid_obstacle_list)
    print(path)

    # convert to real values
    traj = np.zeros((len(path), 2))
    for i in range(len(path)):
        traj[i, 0] = path[i][0] / (N - 1) * (x_max - x_min) + x_min
        traj[i, 1] = path[i][1] / (N - 1) * (y_max - y_min) + y_min

    traj_plt = ax.plot(
        traj[:, 0], traj[:, 1],
        linewidth=0.2,
        color=(.1, .1, .1),
        alpha=1.,
        zorder=2
    )[0]

    # If you want to add the field
    # field_plt = ax.quiver(X, Y, V[:, :, 0], V[:, :, 1])

    plt.subplots_adjust(
        left=0, bottom=0, right=1, top=1, wspace=0, hspace=0
    )
    plt.title("Clean path visualization")

    plt.savefig("outputs/local.pdf", dpi=300)
    plt.show()

if __name__ == "__main__":
    N = 10
    # camera_list = [[np.array([-2., 2.]), -np.pi/4, 0.8]]
    # camera_list = [[np.array([-2, -2.]), 1., .5], [np.array([-2, -2.]), .6, .5]]
    camera_list = []
    obstacle_list = [[grid_to_real_coordinates(N//2, N, 2.), grid_to_real_coordinates(N//2, N, 2.), grid_to_real_length(2, N, 2.)]]
    visualization(N, camera_list, obstacle_list)