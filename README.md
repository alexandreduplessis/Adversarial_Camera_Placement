# Adversarial Path Planning for Optimal Camera Positioning
_Gaia Carenini, Alexandre Duplessis_

This is the code for paper [_Adversarial Path Planning for Optimal Camera Positioning_](https://arxiv.org/abs/2302.07051).

**Note:** This code is currently being written, and is not at all optimized in terms of complexity.

## Abstract
<p style='text-align: justify;'>
The use of visual sensors is  flourishing, driven among others by the several applications in detection and prevention of crimes or dangerous events. While the problem of optimal camera placement for total coverage has been solved for a decade or so, that of the arrangement of cameras maximizing the recognition of objects "in-transit" is still open. The objective of this paper is to attack this problem by providing an adversarial method of proven optimality based on the resolution of Hamilton-Jacobi equations. The problem is attacked by  first assuming the perspective of an adversary, i.e. computing explicitly the path minimizing the probability of detection and the quality of reconstruction. Building on this result,  we introduce an optimality measure for camera configurations and perform a simulated annealing algorithm to find the optimal camera placement.
 </p>

### Requirements
`numpy`, `matplotlib`, `scipy`, `tqdm`

### Usage
- `python3 upwind.py`: Applies Ordered Upwind Algorithm to find minimum time matrix $U$.
- `python3 path_finder.py`: Uses `upwind` output to find constrained optimal path (add option `--visualize 1` to visualize path on grid, `--N n` to fix $N$ to $n$, and `--saveU` to visualize and save $U$).
- `python3 visualization.py`: Tool for visualizing `path_finder` properly and fast (outputs in `local.pdf`).
- `python3 simulated_annealing`: executes main file, optimizing Camera Placement (add option `--visualize 1` to visualize the optimal placement, and `--nbcameras` to set the number of cameras to place).

The folder `draft` contains some older experimental files that will be later deleted (but can still be used for visualization purposes, especially for the Ordered Upwind method).
