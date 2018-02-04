#! /usr/bin/env python

import sys
from graphSerialization import unserializeGraph, serializeGraph, removeDebugFlags


def generateBenchmark(src, dst):
    g = unserializeGraph(src, source=False)
    removeDebugFlags(g)
    del(g["csp"])
    serializeGraph(g, dst, write=True)

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: {0} <source> <destination>".format(sys.argv[0]))
    else:
        src = sys.argv[1]
        dst = sys.argv[2]
        generateBenchmark(src, dst)
