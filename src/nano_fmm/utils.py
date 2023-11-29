from __future__ import annotations

import numpy as np

from ._core.utils import *  # noqa: F403


def bbox2llas(bbox: np.ndarray, *, alt: float = 0.0):
    lon0, lat0, lon1, lat1 = bbox
    return np.array(
        [
            [lon0, lat0, alt],
            [lon1, lat0, alt],
            [lon1, lat1, alt],
            [lon0, lat1, alt],
            [lon0, lat0, alt],
        ]
    )
