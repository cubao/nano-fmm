import numpy as np

import nano_fmm as fmm
from nano_fmm import LineSegment


def test_add():
    assert fmm.add(1, 2) == 3


def test_segment():
    seg = LineSegment([0, 0, 0], [10, 0, 0])
    assert 4.0 == seg.distance([5.0, 4.0, 0.0])
    assert 5.0 == seg.distance([-4.0, 3.0, 0.0])
    assert 5.0 == seg.distance([14.0, 3.0, 0.0])
    assert 25.0 == seg.distance2([14.0, 3.0, 0.0])

    assert seg.length == 10.0
    assert seg.length2 == 100.0
    assert np.all(seg.A == [0, 0, 0])
    assert np.all(seg.B == [10, 0, 0])
    assert np.all(seg.AB == [10, 0, 0])
    assert np.all(seg.dir == [1, 0, 0])
    assert np.all(seg.interpolate(0.4) == [4, 0, 0])
    assert seg.t([4, 0, 0]) == 0.4
    PP, dist, t = seg.nearest([4, 1, 0])
    assert np.all(PP == [4, 0, 0])
    assert dist == 1.0
    assert t == 0.4

    seg = LineSegment([0, 0, 0], [0, 0, 0])
    assert 5.0 == seg.distance([3.0, 4.0, 0.0])
    assert 5.0 == seg.distance([-4.0, 3.0, 0.0])
    assert 13.0 == seg.distance([5.0, 12.0, 0.0])


def test_utils():
    k0 = fmm.utils.cheap_ruler_k(0.0)
    k1 = fmm.utils.cheap_ruler_k(30.0)
    assert k0[0] > k1[0]

    anchor = [123.4, 5.6, 7.8]
    enus = [[1, 2, 3], [4, 5, 6]]
    llas = fmm.utils.enu2lla(enus, anchor_lla=anchor)
    enus2 = fmm.utils.lla2enu(llas, anchor_lla=anchor)
    assert np.max(enus2 - enus) < 1e-9


def test_polyline():
    enus = [[0, 0, 0], [10, 0, 0], [13, 4, 0]]
    fmm.Polyline(enus)
    print()

for i in range(90):
    print(f'K({i})', end=',')

test_polyline()
print()
