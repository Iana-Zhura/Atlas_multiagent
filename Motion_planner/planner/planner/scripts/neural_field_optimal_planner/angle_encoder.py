import numpy as np
import torch
from torch import nn


class AngleEncoder(nn.Module):
    def __init__(self, encoding_dimension=10):
        super().__init__()
        self._encoding_dimension = encoding_dimension
        self._biases = nn.Parameter(torch.zeros(2 * self._encoding_dimension), requires_grad=True)
        frequencies = torch.linspace(1, encoding_dimension, encoding_dimension)
        self._frequencies = nn.Parameter(torch.cat([frequencies, frequencies], dim=0), requires_grad=False)
        nn.init.uniform_(self._biases, -np.pi, np.pi)

    def forward(self, x):
        x = (x[:, None] + self._biases[None]) * self._frequencies[None]
        x = torch.cat([torch.sin(x[:, :self._encoding_dimension]), torch.cos(x[:, self._encoding_dimension:])], dim=1)
        return x

    @property
    def encoding_dimension(self):
        return 2 * self._encoding_dimension
