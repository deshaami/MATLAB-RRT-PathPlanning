# MATLAB RRT* Robot Navigation Simulation

This project is a **MATLAB-based 2D robot navigation simulation** using **RRT*** path planning and mild **reactive obstacle avoidance** with LIDAR. The robot navigates from a user-defined start to goal on a floor-plan image.

---

## Features

- Loads any **floor-plan image** (PNG/JPG/BMP) and converts it into a binary occupancy map.
- User selects **start and goal positions** interactively.
- **RRT*** algorithm finds a feasible path.
- **Pure pursuit controller** for smooth path following.
- Simulated **LIDAR** for obstacle detection and mild reactive avoidance.
- Saves simulation as a **video (MP4)** for visualization.
- Path is smoothed for realistic robot motion.

---

## Prerequisites

- MATLAB 2018b or later (tested)
- Image Processing Toolbox recommended

---

## How to Run

1. Open MATLAB and navigate to the project folder.
2. Run the main function:

```matlab
phase1_demo_v5_fixed
