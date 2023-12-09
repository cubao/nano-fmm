from __future__ import annotations

import contextlib
import io
import json
import os
import sys
import tempfile
import time
from collections import defaultdict
from contextlib import contextmanager

import numpy as np
import pytest

import nano_fmm as fmm
from nano_fmm import LineSegment, Network, rapidjson
from nano_fmm import flatbush as fb

__PWD = os.path.abspath(os.path.dirname(__file__))
__BUILD = os.path.abspath(f"{__PWD}/../build")
os.makedirs(__BUILD, exist_ok=True)


def test_add():
    assert fmm.add(1, 2) == 3


def test_segment():
    seg = LineSegment([0, 0, 0], [10, 0, 0])
    assert seg.distance([5.0, 4.0, 0.0]) == 4.0
    assert seg.distance([-4.0, 3.0, 0.0]) == 5.0
    assert seg.distance([14.0, 3.0, 0.0]) == 5.0
    assert seg.distance2([14.0, 3.0, 0.0]) == 25.0

    assert seg.length == 10.0
    assert seg.length2 == 100.0
    assert np.all([0, 0, 0] == seg.A)
    assert np.all([10, 0, 0] == seg.B)
    assert np.all([10, 0, 0] == seg.AB)
    assert np.all(seg.dir == [1, 0, 0])
    assert np.all(seg.interpolate(0.4) == [4, 0, 0])
    assert seg.t([4, 0, 0]) == 0.4
    PP, dist, t = seg.nearest([4, 1, 0])
    assert np.all([4, 0, 0] == PP)
    assert dist == 1.0
    assert t == 0.4

    seg = LineSegment([0, 0, 0], [0, 0, 0])
    assert seg.distance([3.0, 4.0, 0.0]) == 5.0
    assert seg.distance([-4.0, 3.0, 0.0]) == 5.0
    assert seg.distance([5.0, 12.0, 0.0]) == 13.0

    seg = LineSegment([0, 0, 0], [0, 0, 0])
    assert seg.length == 0.0
    assert seg.length2 == 0.0
    assert seg.distance([0, 1, 0]) == 1.0
    pt, d, t = seg.nearest([1, 0, 0])
    assert np.all(pt == [0, 0, 0])
    assert d == 1.0
    assert t == 0.0


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
        assert np.max(np.fabs(seg1.A - seg2.A)) < 1e-2
        assert np.max(np.fabs(seg1.B - seg2.B)) < 1e-2


def test_cheap_ruler_k():
    N = 100000
    tic = time.time()
    fmm.benchmarks.cheap_ruler_k(N)
    toc = time.time()
    delta = toc - tic
    print(delta, "secs")
    tic = time.time()
    fmm.benchmarks.cheap_ruler_k_lookup_table(N)
    toc = time.time()
    delta2 = toc - tic
    print(delta2, "secs (with lookup)", f"speed up x{delta/delta2:.3f}")
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
    assert len(tree.to_bytes()) == 120

    bboxes = [
        [0, 0, 10, 10],
        [1, 1, 5, 5],
        [3, 3, 7, 7],
        [2, 2, 9, 3],
    ]
    bboxes = np.array(bboxes, dtype=np.float64)
    tree1 = fb.PackedRTree(bboxes[:, :2], bboxes[:, 2:])
    tree2 = fb.PackedRTree(
        fb.hilbertSort([fb.NodeItem(*bbox, idx) for idx, bbox in enumerate(bboxes)]),
        extent=fb.NodeItem(0, 0, 10, 10),
    )
    for tree in [tree1, tree2]:
        assert tree.getExtent().to_numpy().tolist() == [0, 0, 10, 10]
        assert tree.getNumItems() == 4
        assert tree.getNumNodes() == 5
        assert tree.getNodeSize() == 16
        data = tree.to_bytes()
        assert isinstance(data, bytes)
        assert tree.size() == len(data) == 200
        assert len(tree.search(0, 0, 3, 3)) == 4
        assert tree.searchIndex(0, 0, 1, 1).tolist() == [0, 1]
        assert tree.searchIndex(0, 0, 0.1, 0.1).tolist() == [0]


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
    assert nodes[0].intersects(fb.NodeItem(0, 0, 0, 0))
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
    assert isinstance(data, bytes)

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


def test_polyline_nearest_slice():
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

    pt, dist, seg_idx, t = polyline.nearest([1.5, 0, 0])
    assert np.all(pt == [1.5, 0, 0])
    assert dist == 0.0
    assert seg_idx == 0
    assert t == 0.5

    pt, dist, seg_idx, t = polyline.nearest([1.5, 3, 0])
    assert np.all(pt == [1.5, 0, 0])
    assert dist == 3.0
    assert seg_idx == 0
    assert t == 0.5

    pt, dist, seg_idx, t = polyline.nearest([1.5, 3, 0], seg_min=1)
    assert np.all(pt == [3, 0, 0])
    assert np.fabs(dist - np.linalg.norm(pt - [1.5, 3, 0])) < 1e-9
    assert seg_idx == 1
    assert t == 0.0

    pt, dist, seg_idx, t = polyline.nearest([5, 0, 0], seg_max=0)
    assert np.all(pt == [3, 0, 0])
    assert dist == 2.0
    assert seg_idx == 0
    assert t == 1.0

    assert np.all(polyline.slice(min=15) == [enus[-1], enus[-1]])
    assert len(polyline.slice(min=16)) == 0

    anchor = [123.4, 5.6, 7.8]
    llas = fmm.utils.enu2lla([*enus, [1.5, 3.0, 0.0]], anchor_lla=anchor)
    polyline = fmm.Polyline(llas[:-1], is_wgs84=True)
    pt, dist, seg_idx, t = polyline.nearest(llas[-1])
    assert np.fabs(pt - (llas[0] + llas[1]) / 2.0).max() < 1e-18
    assert np.fabs(dist - 3.0) < 1e-3
    assert seg_idx == 0
    assert abs(t - 0.5) < 1e-3

    pt, dist, seg_idx, t = polyline.nearest(llas[-1], seg_min=1)
    assert np.all(pt == llas[1])
    assert dist
    assert seg_idx == 1
    assert t == 0.0


def build_network(
    *,
    nodes: dict[str, np.ndarray],
    ways: dict[str, list[str]],
    is_wgs84: bool = False,
):
    node2nexts = defaultdict(list)
    for w, nn in ways.items():
        node2nexts[nn[0]].append(w)
    {nn[-1]: w for w, nn in ways.items()}
    way_ids = dict(zip(ways.keys(), range(len(ways))))
    roads = {}
    for w, nn in ways.items():
        assert len(nn) >= 2
        coords = np.array([nodes[n] for n in nn], dtype=np.float64)
        roads[w] = coords
    network = Network(is_wgs84=is_wgs84)
    for rid, coords in zip(way_ids.values(), roads.values()):
        assert network.add_road(coords, id=rid)
    for rid, nn in ways.items():
        curr_road = way_ids[rid]
        next_roads = node2nexts.get(nn[-1], [])
        # print(f"curr road: '{rid}', nexts: {next_roads}")
        next_roads = [way_ids[r] for r in next_roads]
        for n in next_roads:
            assert network.add_link(curr_road, n)
    return network, {
        "roads": roads,
        "nodes": nodes,
        "ways": ways,
        "way_ids": way_ids,
    }


def two_way_streets(ways: dict[str, list[str]]):
    return {
        **ways,
        **({w[::-1]: nn[::-1] for w, nn in ways.items()}),
    }


def ubodt2json(row, way_ids):
    return {
        "source": way_ids[row.source_road],
        "target": way_ids[row.target_road],
        "next": way_ids[row.source_next],
        "prev": way_ids[row.target_prev],
        "cost": row.cost,
    }


def test_dijkstra():
    """
             E                   F
              o------------------o
           /  |                  |
         /    |                  |
      /       |                  |
    o---------o------------------o---------------o
    A         B                  C               D
    """
    nodes = {
        "A": [0, 0, 0],
        "B": [1, 0, 0],
        "C": [3, 0, 0],
        "D": [5, 0, 0],
        "E": [1, 1, 0],
        "F": [3, 1, 0],
    }
    ways = {
        "AB": ["A", "B"],
        "BC": ["B", "C"],
        "CD": ["C", "D"],
        "AE": ["A", "E"],
        "BE": ["B", "E"],
        "EF": ["E", "F"],
        "CF": ["C", "F"],
    }
    network, meta = build_network(
        nodes=nodes,
        ways=ways,
    )
    nodes, ways, wid2index = (meta[k] for k in ["nodes", "ways", "way_ids"])
    way_ids = list(wid2index.keys())

    rows = network.build_ubodt([wid2index["AB"]], thresh=1)
    rows = [ubodt2json(r, way_ids) for r in sorted(rows)]
    assert rows == [
        {"source": "AB", "target": "BC", "next": "BC", "prev": "AB", "cost": 0.0},
        {"source": "AB", "target": "BE", "next": "BE", "prev": "AB", "cost": 0.0},
        {"source": "AB", "target": "EF", "next": "BE", "prev": "BE", "cost": 1.0},
    ]
    rows = network.build_ubodt([wid2index["BC"]], thresh=5)
    rows = [ubodt2json(r, way_ids) for r in sorted(rows)]
    assert rows == [
        {"source": "BC", "target": "CD", "next": "CD", "prev": "BC", "cost": 0.0},
        {"source": "BC", "target": "CF", "next": "CF", "prev": "BC", "cost": 0.0},
    ]

    network, meta = build_network(
        nodes=nodes,
        ways=two_way_streets(ways),
    )
    nodes, ways, wid2index = (meta[k] for k in ["nodes", "ways", "way_ids"])
    way_ids = list(wid2index.keys())

    rows = network.build_ubodt([wid2index["BC"]], thresh=3)
    rows = [ubodt2json(r, way_ids) for r in sorted(rows)]
    assert rows == [
        {"source": "BC", "target": "CD", "next": "CD", "prev": "BC", "cost": 0.0},
        {"source": "BC", "target": "CF", "next": "CF", "prev": "BC", "cost": 0.0},
        {"source": "BC", "target": "CB", "next": "CB", "prev": "BC", "cost": 0.0},
        {"source": "BC", "target": "FE", "next": "CF", "prev": "CF", "cost": 1.0},
        {"source": "BC", "target": "FC", "next": "CF", "prev": "CF", "cost": 1.0},
        {"source": "BC", "target": "DC", "next": "CD", "prev": "CD", "cost": 2.0},
        {"source": "BC", "target": "BE", "next": "CB", "prev": "CB", "cost": 2.0},
        {"source": "BC", "target": "BA", "next": "CB", "prev": "CB", "cost": 2.0},
        {"source": "BC", "target": "EF", "next": "CF", "prev": "FE", "cost": 3.0},
        {"source": "BC", "target": "EA", "next": "CF", "prev": "FE", "cost": 3.0},
        {"source": "BC", "target": "EB", "next": "CF", "prev": "FE", "cost": 3.0},
        {"source": "BC", "target": "AB", "next": "CB", "prev": "BA", "cost": 3.0},
        {"source": "BC", "target": "AE", "next": "CB", "prev": "BA", "cost": 3.0},
    ]


def test_random_stroke():
    for _ in range(3):
        rc = fmm.RandomColor(0)
        stroke = rc.next_hex()
        assert stroke == "#38b5e9"
        rc = fmm.RandomColor()
        stroke = rc.next_hex()
        assert stroke != "#38b5e9"


@contextmanager
def capture_and_discard_output():
    stdout_fileno = sys.stdout.fileno()
    stderr_fileno = sys.stderr.fileno()
    saved_stdout_fileno = os.dup(stdout_fileno)
    saved_stderr_fileno = os.dup(stderr_fileno)

    with tempfile.NamedTemporaryFile(mode="w+") as tempf:
        try:
            os.dup2(tempf.fileno(), stdout_fileno)
            os.dup2(tempf.fileno(), stderr_fileno)
            yield tempf
        finally:
            os.dup2(saved_stdout_fileno, stdout_fileno)
            os.dup2(saved_stderr_fileno, stderr_fileno)
            os.close(saved_stdout_fileno)
            os.close(saved_stderr_fileno)


def test_logging():
    fmm.utils.logging("hello one")
    fmm.utils.set_logging_level(4)
    fmm.utils.logging("hello three")
    with fmm.utils.ostream_redirect(stdout=True, stderr=True):
        fmm.utils.logging("hello four")
    buffer = io.StringIO()
    with contextlib.redirect_stdout(buffer):
        print("This will be captured")
    output_string = buffer.getvalue()
    assert output_string == "This will be captured\n"

    buffer = io.StringIO()
    with contextlib.redirect_stdout(buffer), contextlib.redirect_stderr(
        buffer
    ), fmm.utils.ostream_redirect(stdout=True, stderr=True):
        fmm.utils.logging("hello five")
    output_string = buffer.getvalue()
    assert output_string == "std::cout: hello five\nstd::cerr: hello five\n"

    # fmm.utils.setup()

    buffer = io.StringIO()
    with contextlib.redirect_stdout(buffer), contextlib.redirect_stderr(buffer):
        fmm.utils.flush()
        with fmm.utils.ostream_redirect(stdout=True, stderr=True):
            fmm.utils.logging("hello six")
            fmm.utils.flush()
        fmm.utils.flush()
    output_string = buffer.getvalue()
    # assert output_string == "std::cout: hello five\nstd::cerr: hello five\n"

    # with capture_and_discard_output() as output:
    #     print("hello world")
    #     fmm.utils.logging("hello seven")
    #     fmm.utils.flush()
    #     output.seek(0)
    #     output.read()
    # fmm.utils.logging("hello eight")
    # print(f"Captured: {output}")


def test_json():
    j = rapidjson()
    assert j.dumps() == "null"
    assert json.dumps(None) == "null"
    j = rapidjson({})
    assert j.dumps() == "{}"
    j = rapidjson([])
    assert j.dumps() == "[]"
    assert rapidjson(5).dumps() == "5"
    assert rapidjson(3.14).dumps() == "3.14"
    assert rapidjson("text").dumps() == '"text"'
    for text in [
        "3.14",
        "5",
        '"text"',
        '{"key": "value"}',
        '["list", "items"]',
    ]:
        assert rapidjson().loads(text)() == json.loads(text)


def test_project_point_rapidjson():
    pt = fmm.ProjectedPoint()
    with pytest.raises(Exception) as excinfo:  # noqa: PT011
        pt.position[0] = 5
    assert "read-only" in str(excinfo.value)
    j = pt.to_rapidjson()
    assert j() == {
        "position": [0.0, 0.0, 0.0],
        "direction": [0.0, 0.0, 1.0],
        "distance": 0.0,
        "road_id": 0,
        "offset": 0.0,
    }


def test_ubodt_rapidjson():
    rec = fmm.UbodtRecord()
    j = rec.to_rapidjson()
    assert j() == {
        "source_road": 0,
        "target_road": 0,
        "source_next": 0,
        "target_prev": 0,
        "cost": 0.0,
    }
    j["source_road"] = 666
    rec.from_rapidjson(j)
    assert rec.source_road == 666


def test_indexer():
    indexer = fmm.Indexer()
    assert indexer.id(5) == "5"
    assert indexer.id(10) == "10"
    assert indexer.id(1000) == "1000"
    assert indexer.id("1000") == 1000
    assert indexer.to_rapidjson()() == {
        "10": 10,
        "1000": 1000,
        "5": 5,
    }

    indexer = fmm.Indexer()
    assert indexer.id("road1") == 1000000
    assert indexer.id("road2") == 1000001
    assert indexer.id("road3") == 1000002
    assert indexer.id("road2") == 1000001
    assert indexer.id("13579") == 13579
    assert indexer.id(1000002) == "road3"
    assert indexer.id("1000002") == 1000003
    assert indexer.id("1000005") == 1000005
    assert indexer.to_rapidjson()() == {
        "1000002": 1000003,
        "1000005": 1000005,
        "13579": 13579,
        "road1": 1000000,
        "road2": 1000001,
        "road3": 1000002,
    }

    indexer2 = fmm.Indexer().from_rapidjson(indexer.to_rapidjson())
    assert indexer2.id("road1") == 1000000
    assert indexer2.id("road2") == 1000001
    assert indexer2.id("road3") == 1000002
    assert indexer2.id("road2") == 1000001
    assert indexer2.id("13579") == 13579
    assert indexer2.id(1000002) == "road3"
    assert indexer2.id("1000002") == 1000003
    assert indexer2.id("1000005") == 1000005
    assert indexer2.to_rapidjson() == indexer.to_rapidjson()
    indexer2.id("add another road")
    assert indexer2.to_rapidjson() != indexer.to_rapidjson()


# fmm.utils.get_logging_level()
# fmm.utils.set_logging_level(0)  # trace
# # fmm.utils.set_logging_level(6) # off


def test_network_read_write():
    network = Network.load(f"{__PWD}/README.md")
    assert network is None
    network = Network.load("missing_file")
    assert network is None

    from nano_fmm.converter import remap_network_with_string_id

    geojson, _ = remap_network_with_string_id(f"{__PWD}/../data/suzhoubeizhan.json")
    network = Network(is_wgs84=True)
    network.from_geojson(geojson)
    assert len(network.roads()) == 1016
    assert network.to_geojson().dump(f"{__BUILD}/network.geojson", indent=True)
    assert network.to_rapidjson().dump(f"{__BUILD}/network.json", indent=True)

    network = Network.load(f"{__BUILD}/network.geojson")
    network.to_geojson().dump(f"{__BUILD}/network2.geojson", indent=True)
    network.to_rapidjson().dump(f"{__BUILD}/network2.json", indent=True)

    network = Network.load(f"{__BUILD}/network.json")
    network.to_geojson().dump(f"{__BUILD}/network3.geojson", indent=True)
    network.to_rapidjson().dump(f"{__BUILD}/network3.json", indent=True)

    rows = network.build_ubodt()
    rows = sorted(rows)
    print(rows[:5])


def test_network_query():
    path = f"{__BUILD}/network.geojson"
    if not os.path.isfile(path):  # noqa: PTH113
        test_network_read_write()
    network = Network.load(path)
    assert network.is_wgs84()
    assert len(network.roads()) == 1016
    assert isinstance(network.roads(), list)
    assert network.next_roads(1293) == [1297, 1298]
    assert isinstance(network.next_roads(1293), list)
    assert network.prev_roads(1297) == [1293]

    assert network.has_road(1293)
    assert not network.has_road(-1)
    assert network.has_link(1293, 1297)
    assert not network.has_link(1293, 1299)

    assert network.remove_link(1293, 1298)
    assert not network.remove_link(1293, 1298)
    assert network.next_roads(1293) == [1297]

    polyline = network.road(1293)
    assert polyline.is_wgs84()
    assert len(polyline.as_numpy()) == polyline.N() == 6
    assert round(polyline.length(), 3) == 225.543
    assert round(polyline.range(0, t=0.3), 3) == 10.236
    assert polyline.k().round(2).tolist() == [95504.26, 110869.46, 1.0]
    seed_lla = [120.663031, 31.40531, 0]
    lla, dist, seg_idx, t = polyline.nearest(seed_lla)
    assert round(dist, 2) == 105.71
    assert seg_idx == 4
    assert t == 1.0
    # 0---1---2---3---4---5
    #                     ^
    #                   t=1.0
    assert np.fabs(polyline.range(4, t=1.0) - polyline.length()) < 1e-15

    hits = network.query(seed_lla, radius=40.0)
    assert len(hits) == 4


def test_network_query_enu():
    network = Network(is_wgs84=False)
    """
    r0 o-------------o
                     |
    r1 o-------------o
                     |
    r2               o
    """
    network.add_road([[0, 5, 0], [10, 5, 0]], id=0)
    network.add_road([[0, 0, 0], [10, 0, 0]], id=1)
    network.add_road([[10, -5, 0], [10, 5, 0]], id=2)
    hits = network.query([5, 0, 0], radius=3)
    hits = [h.to_rapidjson()() for h in hits]
    assert hits == [
        {
            "position": [5.0, 0.0, 0.0],
            "direction": [1.0, 0.0, 0.0],
            "distance": 0.0,
            "road_id": 1,
            "offset": 5.0,
        },
    ]

    hits = network.query([5, 0, 0], radius=5)
    hits = [h.to_rapidjson()() for h in hits]
    hit1 = {
        "position": [5.0, 5.0, 0.0],
        "direction": [1.0, 0.0, 0.0],
        "distance": 5.0,
        "road_id": 0,
        "offset": 5.0,
    }
    hit2 = {
        "position": [10.0, 0.0, 0.0],
        "direction": [0.0, 1.0, 0.0],
        "distance": 5.0,
        "road_id": 2,
        "offset": 5.0,
    }
    assert hits[0] == {
        "position": [5.0, 0.0, 0.0],
        "direction": [1.0, 0.0, 0.0],
        "distance": 0.0,
        "road_id": 1,
        "offset": 5.0,
    }
    assert hits[1:] == [hit1, hit2] or hits[1:] == [hit2, hit1]


def pytest_main(dir: str, *, test_file: str):
    # pytest test_cli.py
    # pytest --capture=tee-sys test_cli.py
    os.chdir(dir)
    # https://docs.pytest.org/en/6.2.x/usage.html#calling-pytest-from-python-code
    sys.exit(
        pytest.main(
            [
                dir,
                *(["-k", test_file] if test_file else []),
                "--capture",
                "tee-sys",
                "-vv",
                "-x",
            ]
        )
    )


if __name__ == "__main__":
    pytest_main(__PWD, test_file=os.path.basename(__file__))  # noqa: PTH119
