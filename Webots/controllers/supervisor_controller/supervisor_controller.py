#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Dec  8 13:12:27 2025

@author: vedasaipolisetty
"""

from controller import Supervisor
import math
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import os

# ---------------- PARAMETERS ----------------
TIME_STEP = 32
dt = TIME_STEP / 1000.0
SIM_TIME = 120.0
STEPS = int(SIM_TIME / dt)

NUM_ROBOTS = 5
COMM_RADIUS = 2.5

WORLD_HALF = 5.0      # 10x10 arena
MAP_RES = 0.1
MAP_SIZE = int((2 * WORLD_HALF) / MAP_RES)

UNKNOWN = -1
FREE = 0

RESULTS_DIR = "results"
os.makedirs(RESULTS_DIR, exist_ok=True)

# ---------------- SUPERVISOR INIT ----------------
supervisor = Supervisor()

robot_ids = ["R1", "R2", "R3", "R4", "R5"]
robots = [supervisor.getFromDef(rid) for rid in robot_ids]

local_maps = [UNKNOWN * np.ones((MAP_SIZE, MAP_SIZE)) for _ in range(NUM_ROBOTS)]
trajectories = [[] for _ in range(NUM_ROBOTS)]

coverage_history = []
connectivity_history = []
min_distance_history = []

# ---------------- UTILS ----------------
def world_to_map(x, z):
    ix = int((x + WORLD_HALF) / MAP_RES)
    iz = int((z + WORLD_HALF) / MAP_RES)

    ix = min(max(ix, 0), MAP_SIZE - 1)
    iz = min(max(iz, 0), MAP_SIZE - 1)

    return ix, iz

# ---------------- MAIN LOOP ----------------
for _ in range(STEPS):
    if supervisor.step(TIME_STEP) == -1:
        break

    positions = []

    # --- Position + Mapping ---
    for i, robot in enumerate(robots):
        pos = robot.getPosition()
        x, z = pos[0], pos[2]

        positions.append([x, z])
        trajectories[i].append([x, z])

        ix, iz = world_to_map(x, z)
        local_maps[i][ix, iz] = FREE

    positions = np.array(positions)

    # --- Connectivity + Min Distance ---
    edges = 0
    min_dist = float("inf")

    for i in range(NUM_ROBOTS):
        for j in range(i + 1, NUM_ROBOTS):
            d = np.linalg.norm(positions[i] - positions[j])

            if d < COMM_RADIUS:
                edges += 1

            min_dist = min(min_dist, d)

    connectivity_history.append(edges)
    min_distance_history.append(min_dist)

    # --- Coverage ---
    global_map = UNKNOWN * np.ones((MAP_SIZE, MAP_SIZE))
    for m in local_maps:
        known = (m != UNKNOWN)
        global_map[known] = m[known]

    coverage = np.count_nonzero(global_map != UNKNOWN) / global_map.size
    coverage_history.append(coverage)

# ---------------- PLOTTING ----------------
time_vector = np.arange(len(coverage_history)) * dt

plt.plot(time_vector, coverage_history)
plt.title("Coverage Over Time")
plt.xlabel("Time (s)")
plt.ylabel("Coverage")
plt.grid(True)
plt.savefig(os.path.join(RESULTS_DIR, "coverage_over_time.png"))
plt.close()

plt.plot(time_vector, connectivity_history)
plt.title("Connectivity Over Time")
plt.xlabel("Time (s)")
plt.ylabel("Number of Links")
plt.grid(True)
plt.savefig(os.path.join(RESULTS_DIR, "connectivity_over_time.png"))
plt.close()

plt.plot(time_vector, min_distance_history)
plt.title("Minimum Inter-Robot Distance Over Time")
plt.xlabel("Time (s)")
plt.ylabel("Distance (m)")
plt.grid(True)
plt.savefig(os.path.join(RESULTS_DIR, "min_distance_over_time.png"))
plt.close()

plt.figure()
for traj in trajectories:
    traj = np.array(traj)
    plt.plot(traj[:, 0], traj[:, 1])
plt.title("Robot Trajectories")
plt.xlabel("X position (m)")
plt.ylabel("Z position (m)")
plt.axis("equal")
plt.grid(True)
plt.savefig(os.path.join(RESULTS_DIR, "robot_trajectories.png"))
plt.close()

print("Simulation complete. All results saved in the 'results' folder.")
