from nano_fmm import Network

class RoadNetwork(Network):
    pass

if __name__ == "__main__":
    import fire

    fire.core.Display = lambda lines, out: print(*lines, file=out)
    fire.Fire(RoadNetwork)
