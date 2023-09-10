import os
from typing import Union

from loguru import logger

from nano_fmm import Indexer, rapidjson


def remap_network_with_string_id(
    network: Union[str, rapidjson],
    *,
    export: str = None,
):
    """
    network.json normally has:
        {
            "type": "Feature",
            "geometry": {...},
            "properties": {
                "id": 244,
                "nexts": [326, 452],
                "prevs": [5241, 563],
                ...
            },
        }

    if you have:
        {
            "id": "road1",
            "nexts": ["road2", "road3"],
            "prevs": ["road4", "road5"],
        }
    """
    if isinstance(network, str):
        path = network
        network = rapidjson()
        network.load(path)
    indexer = Indexer()
    features = network["features"]
    for i in range(len(features)):
        f = features[i]
        props = f["properties"]
        if "id" not in props or "nexts" not in props and "prevs" not in props:
            continue
        props["id"] = indexer.id(props["id"]())
        props["nexts"] = [indexer.id(n) for n in props["nexts"]()]
        props["prevs"] = [indexer.id(n) for n in props["prevs"]()]
        if "folds" in props:
            props["folds"][0] = [indexer.id(i) for i in props["folds"][0]()]
        props["type"] = "road"
    if export:
        network["index"] = indexer.to_rapidjson()
        export = os.path.abspath(export)
        os.makedirs(os.path.dirname(export), exist_ok=True)
        network.dump(export, indent=True)
        logger.info(f"wrote to {export}")
        return export
    else:
        return network, indexer


def remap_network_to_string_id(
    network: Union[str, rapidjson],
    *,
    export: str = None,
):
    if isinstance(network, str):
        path = network
        network = rapidjson()
        network.load(path)
    features = network["features"]
    for i in range(len(features)):
        f = features[i]
        props = f["properties"]
        if "id" not in props or "nexts" not in props and "prevs" not in props:
            continue
        props["id"] = str(props["id"]())
        props["nexts"] = [str(n) for n in props["nexts"]()]
        props["prevs"] = [str(n) for n in props["prevs"]()]
    if export:
        export = os.path.abspath(export)
        os.makedirs(os.path.dirname(export), exist_ok=True)
        network.dump(export, indent=True)
        logger.info(f"wrote to {export}")
        return export
    else:
        return network


if __name__ == "__main__":
    import fire

    fire.core.Display = lambda lines, out: print(*lines, file=out)
    fire.Fire(
        {
            "remap_network_str2int": remap_network_with_string_id,
            "remap_network_int2str": remap_network_to_string_id,
        }
    )
