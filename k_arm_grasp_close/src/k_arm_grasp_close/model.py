"""The close-the-gripper network and its saved file."""

import numpy as np
import torch
from torch import nn


class GraspCloseNet(nn.Module):
    """Dilated 1-D convolution over the window, plus the last frame -> logit.

    Parameters
    ----------
    n_features : int
        Features per frame.
    hidden : int
        Channels of the convolution and the head.
    """

    def __init__(self, n_features, hidden=64):
        super(GraspCloseNet, self).__init__()
        self.conv = nn.Sequential(
            nn.Conv1d(n_features, hidden, 3, padding=1), nn.ReLU(),
            nn.Conv1d(hidden, hidden, 3, padding=2, dilation=2), nn.ReLU(),
            nn.Conv1d(hidden, hidden, 3, padding=4, dilation=4), nn.ReLU())
        self.head = nn.Sequential(nn.Linear(2 * hidden + n_features, hidden), nn.ReLU(),
                                  nn.Linear(hidden, 1))

    def forward(self, x):
        """``x``: (batch, window, features), normalised -> (batch,) logits."""
        h = self.conv(x.transpose(1, 2))
        z = torch.cat([h.mean(-1), h[..., -1], x[:, -1]], -1)
        return self.head(z).squeeze(-1)


def save(path, net, cfg, names, mean, std):
    """Write the model file: weights, config, feature names, normaliser."""
    torch.save({'model': net.state_dict(), 'config': cfg, 'feature_names': list(names),
                'mean': np.asarray(mean).tolist(), 'std': np.asarray(std).tolist()}, path)


class GraspCloseModel(object):
    """A saved model, for inference.

    Parameters
    ----------
    path : str
        File written by ``save`` (train.py).
    """

    def __init__(self, path):
        st = torch.load(path, map_location='cpu', weights_only=False)
        self.cfg = st['config']
        self.names = st['feature_names']
        self.mean = np.asarray(st['mean'], np.float32)
        self.std = np.asarray(st['std'], np.float32)
        self.net = GraspCloseNet(len(self.names))
        self.net.load_state_dict(st['model'])
        self.net.eval()

    def prob(self, window):
        """Probability of closing now from one window of frames
        (window, features), unnormalised."""
        x = (np.asarray(window, np.float32) - self.mean) / self.std
        with torch.no_grad():
            return float(torch.sigmoid(self.net(torch.from_numpy(x[None]))).item())
