import numpy as np
import torch.nn.functional
from matplotlib import pyplot as plt


def plot_planner_data(plotted_trajectory, collision_model, boundaries, obstacle_points, device="cpu"):
    plot_model_heatmap(collision_model, boundaries, device)
    plot_obstacle_points(obstacle_points)
    plt.scatter(plotted_trajectory[:, 0], plotted_trajectory[:, 1], color="yellow", s=10)
    plt.tight_layout()


def prepare_figure(boundaries):
    plt.gca().set_aspect("equal")
    plt.gca().axis('off')
    plt.xlim(boundaries[0], boundaries[1])
    plt.ylim(boundaries[2], boundaries[3])


def plot_model_heatmap(model, boundaries, device):
    grid_x, grid_y = np.meshgrid(np.linspace(boundaries[0], boundaries[1], 100),
                                 np.linspace(boundaries[2], boundaries[3], 100))
    grid = np.stack([grid_x, grid_y, np.zeros_like(grid_x)], axis=2).reshape(-1, 3)
    # obstacle_probabilities = nn.functional.softplus(model(torch.tensor(grid.astype(np.float32), device=device)))
    obstacle_probabilities = model(torch.tensor(grid.astype(np.float32), device=device))
    obstacle_probabilities = obstacle_probabilities.cpu().detach().numpy().reshape(100, 100)
    grid = grid.reshape(100, 100, 3)
    # plt.gca().pcolormesh(grid[:, :, 0], grid[:, :, 1], obstacle_probabilities, cmap='RdBu', shading='auto',
    #                      vmin=0, vmax=10)
    plt.gca().pcolormesh(grid[:, :, 0], grid[:, :, 1], obstacle_probabilities, cmap='RdBu', shading='auto')


def plot_nerf_opt_planner(model):
    plt.scatter(model._collision_positions[:, 0], model._collision_positions[:, 1])


def plot_obstacle_points(obstacle_points):
    plt.scatter(obstacle_points[:, 0], obstacle_points[:, 1], color="black")


def plot_positions(positions, color):
    plt.quiver(positions[:, 0], positions[:, 1], np.cos(positions[:, 2]), np.sin(positions[:, 2]), color=color)


def plot_collision_positions(positions, collisions):
    positions = positions.as_vec()
    plot_positions(positions[collisions], "red")
    plot_positions(positions[~collisions], "green")
