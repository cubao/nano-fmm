from _nano_fmm.utils import *
import numpy as np

def bbox2llas(bbox: np.ndarray, *, alt: float = 0.0):
    lon0, lat0, lon1, lat1 = bbox
    return np.array([
        [lon0, lat0, alt],
        [lon1, lat0, alt],
        [lon1, lat1, alt],
        [lon0, lat1, alt],
        [lon0, lat0, alt],
    ])