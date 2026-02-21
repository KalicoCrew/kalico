# Tap Analysis
#
# Copyright (C) 2025  Gareth Farrington <gareth@waves.ky>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import numpy as np


class TapAnalysis:
    def __init__(self, samples):
        nd_samples = np.asarray(samples, dtype=np.float64)
        self.time = nd_samples[:, 0]
        self.force = nd_samples[:, 1]

    # convert to dictionary for JSON encoder
    def to_dict(self):
        return {
            "time": self.time.tolist(),
            "force": self.force.tolist(),
            "is_valid": True,
        }
