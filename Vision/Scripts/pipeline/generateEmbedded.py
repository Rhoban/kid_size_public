#! /usr/bin/env python

import sys
from graphSerialization import unserializeGraph, serializeGraph, removeDebugFlags, iv

configFile="config/configEmbedded.xml"
configGraph = unserializeGraph(configFile)

def generateEmbedded(src, dst):
    g = unserializeGraph(src, source=False)
    for a in g.attributes():
        del g[a]
    for a in configGraph.attributes():
        g[a] = configGraph[a]

    removeDebugFlags(g)
    g["csp"] = { "className" : "SchedulerRequester"}
    g["playing"] = "true"
    g["embedded"] = "true"

    src = configGraph.vs[iv("source", configGraph)]

    g.add_vertex("source")
    node = g.vs[iv("source", g)]
    node["varSet"] = src["varSet"]

    serializeGraph(g, dst, write=True)



if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: {0} <source> <destination>".format(sys.argv[0]))
    else:
        src = sys.argv[1]
        dst = sys.argv[2]
        generateEmbedded(src, dst)
