import numpy as np
import torch


@torch.jit.script
def wrap_angle(angles):
    return (angles + np.pi) % (2 * np.pi) - np.pi
