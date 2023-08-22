import time

import numpy as np

import nano_fmm as fmm
from nano_fmm import LineSegment
from nano_fmm import flatbush as fb


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
    polyline = fmm.Polyline(enus)
    assert polyline.segment(-1) == polyline.segment(1)
    assert polyline.segment(-1) != polyline.segment(0)
    assert np.all(polyline.ranges() == [0, 10, 15])

    anchor = [123.4, 5.6, 7.8]
    k = fmm.utils.cheap_ruler_k(anchor[1])
    llas = fmm.utils.enu2lla(enus, anchor_lla=anchor)
    polyline2 = fmm.Polyline(llas, is_wgs84=True)
    assert np.fabs(polyline2.k() - k).max() < 100
    for i in range(2):
        seg1 = polyline.segment(i)
        seg2 = polyline2.segment(i)
        assert np.max(np.fabs(seg1.A - seg2.A)) < 1e-9
        assert np.max(np.fabs(seg1.B - seg2.B)) < 1e-9


def test_cheap_ruler_k():
    N = 100000
    tic = time.time()
    fmm.benchmarks.cheap_ruler_k(N)
    toc = time.time()
    print(toc - tic, "secs")
    tic = time.time()
    fmm.benchmarks.cheap_ruler_k_lookup_table(N)
    toc = time.time()
    print(toc - tic, "secs (with lookup)")
    print()


def test_geobuf_rtree():
    n = fb.NodeItem()
    assert n.minX == n.minY == n.maxX == n.maxY == 0.0
    assert n.offset == 0
    assert n.width() == n.height() == 0.0

    n = fb.NodeItem.sum(fb.NodeItem(0, 1, 2, 3, 4), fb.NodeItem(0, 10, 20, 30, 40))

    tree = fb.PackedRTree(
        [fb.NodeItem(1, 1, 9, 9, 0), fb.NodeItem(5, 5, 8, 8, 0)],
        extent=fb.NodeItem(0, 0, 10, 10, 0),
    )
    assert len(tree.to_bytes())

    bboxes = [
        [0, 0, 10, 10],
        [1, 1, 5, 5],
        [3, 3, 7, 7],
        [2, 2, 9, 3],
    ]
    bboxes = np.array(bboxes, dtype=np.float64)
    tree = fb.PackedRTree(bboxes[:, :2], bboxes[:, 2:])

    tree.search(0, 0, 3, 3)
    tree.searchIndex(0, 0, 1, 1)
    print()


def test_cpp_migrated_1():
    """
    migrated test: https://github.com/flatgeobuf/flatgeobuf/blob/master/src/cpp/test/packedrtree.h
    PackedRTree 2 item one dimension
    """
    nodes = [
        fb.NodeItem(0, 0, 0, 0),
        fb.NodeItem(0, 0, 0, 0),
    ]
    assert nodes[0] == nodes[1]
    extent = fb.calcExtent(nodes)
    assert nodes[0].intersects(fb.NodeItem(0, 0, 0, 0))
    nodes = fb.hilbertSort(nodes)
    offset = 0
    for node in nodes:
        node.offset = offset
        offset += fb.NodeItem._size_()
    tree = fb.PackedRTree(nodes, extent)
    hits = tree.search(0, 0, 0, 0)
    assert len(hits) == 2
    assert nodes[hits[0].index].intersects(0, 0, 0, 0)


def test_cpp_migrated_2():
    """
    migrated test: https://github.com/flatgeobuf/flatgeobuf/blob/master/src/cpp/test/packedrtree.h
    PackedRTree 2 items 2
    """
    nodes = [
        fb.NodeItem(0, 0, 1, 1),
        fb.NodeItem(2, 2, 3, 3),
    ]
    extent = fb.calcExtent(nodes)
    assert nodes[0].intersects(0, 0, 1, 1)
    assert nodes[1].intersects(2, 2, 3, 3)
    nodes = fb.hilbertSort(nodes)
    offset = 0
    for node in nodes:
        node.offset = offset
        offset += fb.NodeItem._size_()
    assert nodes[1].intersects(0, 0, 1, 1)
    assert nodes[0].intersects(2, 2, 3, 3)

    tree = fb.PackedRTree(nodes, extent)
    data = tree.to_bytes()
    assert len(data) == 120
    assert type(data) == bytes

    hits = tree.search(0, 0, 1, 1)
    assert len(hits) == 1
    assert nodes[hits[0].index].intersects(0, 0, 1, 1)


def test_cpp_migrated_3():
    """
    migrated test: https://githuconstexpr bool operator==(point<T> const& lhs, point<T> const& rhs)
    PackedRTree 19 items + roundtrip + streamSearch
    """
    nodes = [
        fb.NodeItem(0, 0, 1, 1),
        fb.NodeItem(2, 2, 3, 3),
        fb.NodeItem(10, 10, 11, 11),
        fb.NodeItem(100, 100, 110, 110),
        fb.NodeItem(101, 101, 111, 111),
        fb.NodeItem(102, 102, 112, 112),
        fb.NodeItem(103, 103, 113, 113),
        fb.NodeItem(104, 104, 114, 114),
        fb.NodeItem(10010, 10010, 10110, 10110),
        fb.NodeItem(10010, 10010, 10110, 10110),
        fb.NodeItem(10010, 10010, 10110, 10110),
        fb.NodeItem(10010, 10010, 10110, 10110),
        fb.NodeItem(10010, 10010, 10110, 10110),
        fb.NodeItem(10010, 10010, 10110, 10110),
        fb.NodeItem(10010, 10010, 10110, 10110),
        fb.NodeItem(10010, 10010, 10110, 10110),
        fb.NodeItem(10010, 10010, 10110, 10110),
        fb.NodeItem(10010, 10010, 10110, 10110),
        fb.NodeItem(10010, 10010, 10110, 10110),
        fb.NodeItem(10010, 10010, 10110, 10110),
    ]
    extent = fb.calcExtent(nodes)
    nodes = fb.hilbertSort(nodes)
    offset = 0
    for node in nodes:
        node.offset = offset
        offset += fb.NodeItem._size_()

    tree = fb.PackedRTree(nodes, extent)
    hits = tree.search(102, 102, 103, 103)
    assert len(hits) == 4
    for h in hits:
        node = nodes[h.index]
        print(h, node)
        assert node.intersects(102, 102, 103, 103)

    data = tree.to_bytes()
    tree2 = fb.PackedRTree(data, len(nodes))
    hits2 = tree2.search(102, 102, 103, 103)
    assert hits == hits2
    assert hits != hits2[::-1]


def test_polyline_snap_slice():
    enus = [[0, 0, 0], [3, 0, 0], [10, 0, 0], [13, 4, 0]]
    polyline = fmm.Polyline(enus)
    assert polyline.range(0) == 0.0
    assert polyline.range(1) == 3.0
    assert polyline.range(2) == 10.0
    assert polyline.range(3) == 15.0
    assert polyline.range(2, t=0.5) == 12.5
    assert polyline.segment_index_t(12.5) == (2, 0.5)
    assert polyline.segment_index_t(-3.0) == (0, -1.0)
    assert polyline.segment_index_t(-6.0) == (0, -2.0)
    assert polyline.segment_index_t(20.0) == (2, 2.0)

    pt, dist, seg_idx, t = polyline.snap([1.5, 0, 0])
    assert np.all(pt == [1.5, 0, 0])
    assert dist == 0.0
    assert seg_idx == 0 and t == 0.5

    pt, dist, seg_idx, t = polyline.snap([1.5, 3, 0])
    assert np.all(pt == [1.5, 0, 0])
    assert dist == 3.0
    assert seg_idx == 0 and t == 0.5

    pt, dist, seg_idx, t = polyline.snap([1.5, 3, 0], seg_min=1)
    assert np.all(pt == [3, 0, 0])
    assert np.fabs(dist - np.linalg.norm(pt - [1.5, 3, 0])) < 1e-9
    assert seg_idx == 1 and t == 0.0
