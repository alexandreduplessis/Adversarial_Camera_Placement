# Adversarial Path Planning for Optimal Camera Positioning
_Gaia Carenini, Alexandre Duplessis_

This is the code for paper [_Adversarial Path Planning for Optimal Camera Positioning_](https://raw.githubusercontent.com/alexandreduplessis/Clean_Robotics_project/master/Camera_Placement_Paper.pdf).

**Note:** This code is currently being written, and is not at all optimized in terms of complexity.

### Requirements
`numpy`, `matplotlib`, `scipy`, `tqdm`

### Usage
- `python3 upwind.py`: Applies Ordered Upwind Algorithm to find minimum time matrix $U$.
- `python3 path_finder.py`: Uses `upwind` output to find constrained optimal path (add option `--visualize 1` to visualize path on grid, `--N n` to fix $N$ to $n$, and `--saveU` to visualize and save $U$).
- `python3 visualization.py`: Tool for visualizing `path_finder` properly and fast (outputs in `local.pdf`).
- `python3 simulated_annealing`: executes main file, optimizing Camera Placement (add option `--visualize 1` to visualize the optimal placement, and `--nbcameras` to set the number of cameras to place).

The folder `draft` contains some older experimental files that will be later deleted (but can still be used for visualization purposes, especially for the Ordered Upwind method).
