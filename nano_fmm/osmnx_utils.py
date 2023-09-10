import json
import os
from collections import defaultdict
from typing import List

import numpy as np
import osmnx as ox


def topological_sort(nodes, nexts):
    def toposort_util(node, visited, stack, scope):
        visited.add(node)
        for neighbor in nexts.get(node, []):
            if neighbor not in scope:
                continue
            if neighbor not in visited:
                toposort_util(neighbor, visited, stack, scope)
        stack.insert(0, node)

    visited = set()
    stack = []
    scope = set(nodes)
    for node in nodes:
        if node not in visited:
            toposort_util(node, visited, stack, scope)
    return tuple(stack)


def deduplicate_points(coords: np.ndarray) -> np.ndarray:
    coords = np.asarray(coords)
    deltas = np.sum(np.fabs(coords[:-1, :2] - coords[1:, :2]), axis=1)
    if np.count_nonzero(deltas) == len(coords) - 1:
        return coords
    indices = np.r_[0, np.where(deltas != 0)[0] + 1]
    return coords[indices]


def pull_map(
    output: str,
    *,
    bbox: List[float] = None,
    center_dist: List[float] = None,
    network_type: str = "drive",
):
    if bbox is not None:
        bbox = 120.6235, 31.40014, 120.6752, 31.4460
        west, south, east, north = bbox
        G = ox.graph_from_bbox(north, south, east, west, network_type=network_type)
    elif center_dist is not None:
        lon, lat = center_dist[:2]
        dist = center_dist[2] if len(center_dist) > 2 else 500.0
        G = ox.graph_from_point((lat, lon), dist=dist, network_type=network_type)
    else:
        raise Exception(
            "should specify --bbox=LEFT,BOTTOM,RIGHT,TOP or --center_dist=LON,LAT,DIST"
        )
        # G = ox.graph_from_address("350 5th Ave, New York, New York", network_type="drive")
        # G = ox.graph_from_place("Los Angeles, California", network_type="drive")

    nodes, edges = ox.graph_to_gdfs(G)
    nodes = ox.io._stringify_nonnumeric_cols(nodes)
    edges = ox.io._stringify_nonnumeric_cols(edges)

    # G = ox.project_graph(G)
    # G = ox.consolidate_intersections(G, tolerance=10 / 1e5, rebuild_graph=True, dead_ends=True)

    edge2llas = {}
    for k, edge in edges.iterrows():
        edge2llas[k[:2]] = np.array(edge["geometry"].coords)
    ways = dict(zip(edge2llas.keys(), range(len(edge2llas))))
    heads, tails = defaultdict(set), defaultdict(set)
    for s, e in edge2llas:
        wid = ways[(s, e)]
        heads[s].add(wid)
        tails[e].add(wid)
    features = []
    for (s, e), geom in edge2llas.items():
        f = {
            "type": "Feature",
            "geometry": {
                "type": "LineString",
                "coordinates": geom.tolist(),
            },
            "properties": {
                "type": "road",
                "id": ways[(s, e)],
                "nexts": sorted(heads[e]),
                "prevs": sorted(tails[s]),
                "nodes": [int(s), int(e)],
            },
        }
        features.append(f)
    geojson = {
        "type": "FeatureCollection",
        "features": features,
    }

    if not output:
        return geojson

    output = os.path.abspath(output)
    os.makedirs(os.path.dirname(output), exist_ok=True)
    with open(output, "w") as f:
        json.dump(geojson, f, indent=4)
    return output


if __name__ == "__main__":
    import fire

    fire.core.Display = lambda lines, out: print(*lines, file=out)
    fire.Fire(
        {
            "pull_map": pull_map,
        }
    )
