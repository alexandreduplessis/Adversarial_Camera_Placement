import numpy as np
import sys, os
import inspect
import ast


def np_belongs(x, L):
    for i in range(len(L)):
        if np.array_equal(x, L[i]):
            return True
    return False

def cost_path(path):
    # sum of the distances between consecutive points
    cost = 0
    for i in range(len(path)-1):
        cost += np.linalg.norm(path[i] - path[i+1])
    return cost

# NF(x) is the set of segments [xj, xk] where [xj, xk] is in AF, and there exists x' on [xj, xk] such that x' is at distance at most nfconstant from x
def compute_NF(x, AF):
    nfconstant = 1.5
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

def exists_considered(M):
    N = M.shape[0]
    for i in range(N):
        for j in range(N):
            if M[i, j] == 1:
                return True, np.array([i,j])
    return False, 0

def compute_AcceptedFront(M, o):
    N = M.shape[0]
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
    N = M.shape[0]
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

def compute_U(x, xj, xk, uj, uk, f, history):
    # if it is (8, 8)
    if x[0] == 8 and x[1] == 8:
        print("x")
        print("xj", xj)
        print("xk", xk)
        print("uj", uj)
        print("uk", uk)
    # check if [x, xj, xk, uj, uk] is in history
    if str([x, xj, xk, uj, uk]) in history:
        return history[str([x, xj, xk, uj, uk])], history
    bnds = ((0, 1),)
    minig = lambda theta: f(x, (theta*xj + (1-theta)*xk - x)/np.linalg.norm(theta*xj + (1-theta)*xk - x))
    g = lambda theta: np.linalg.norm(theta*xj + (1-theta)*xk - x)/f(x, (theta*xj + (1-theta)*xk - x)/np.linalg.norm(theta*xj + (1-theta)*xk - x)) + theta*uj + (1-theta)*uk
    # x_test is 100 random points between 0 and 1
    x_test = np.random.rand(100)
    y_test = []
    for i in range(len(x_test)):
        y_test.append(g(x_test[i]))
        # if minig is zero
        if minig(x_test[i]) == 0.:
            print("error with minig:", x, xj, xk, x_test[i], minig(x_test[i]))
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
    # if it is (8, 8)
    if x[0] == 8 and x[1] == 8:
        print("res", res)
    return res, history