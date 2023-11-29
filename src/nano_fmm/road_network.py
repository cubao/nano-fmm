from __future__ import annotations

from nano_fmm import Network


class RoadNetwork(Network):
    def __init__(self, is_wgs84: bool = False):
        super().__init__(is_wgs84)


if __name__ == "__main__":
    import fire

    fire.core.Display = lambda lines, out: print(*lines, file=out)
    fire.Fire(RoadNetwork)
