import xml.etree.ElementTree as ET
from xml.etree.ElementTree import Element, tostring
import xml.dom.minidom
import huTools.structured as ht
from os.path import basename

from igraph import *
from igraph.drawing.vertex import DefaultVertexDrawer
from igraph.drawing import Plot
import cairo as cairo
import copy


#=========================================================================
def drawR(self, visual_vertex, vertex, coords):
    context = self.context
    visual_vertex.shape.draw_path(
        context, coords[0], coords[1], visual_vertex.size)
    vertex["x"] = coords[0]
    vertex["y"] = coords[1]
    context.set_source_rgba(*visual_vertex.color)
    context.fill_preserve()
    context.set_source_rgba(*visual_vertex.frame_color)
    context.set_line_width(visual_vertex.frame_width)
    context.stroke()


def f(self):
    sur = cairo.ImageSurface(
        cairo.FORMAT_ARGB32, int(self.bbox.width), int(self.bbox.height))
    ctx = cairo.Context(sur)
    self.redraw(ctx)
Plot.show = f
DefaultVertexDrawer.draw = drawR


class XmlListConfig(list):

    def __init__(self, aList):
        for element in aList:
            if len(element):
                if len(element) == 1 or element[0].tag != element[1].tag:
                    self.append(XmlDictConfig(element))
                elif element[0].tag == element[1].tag:
                    self.append(XmlListConfig(element))
            elif element.text:
                text = element.text.strip()
                if text:
                    self.append(text)


class XmlDictConfig(dict):

    def __init__(self, parent_element):
        if parent_element.items():
            self.update(dict(parent_element.items()))
        for element in parent_element:
            if len(element):
                if len(element) == 1 or element[0].tag != element[1].tag:
                    aDict = XmlDictConfig(element)
                else:
                    aDict = {element[0].tag: XmlListConfig(element)}
                if element.items():
                    aDict.update(dict(element.items()))
                self.update({element.tag: aDict})
            elif element.items():
                self.update({element.tag: dict(element.items())})
            else:
                self.update({element.tag: element.text})


#=========================================================================
def prettify(elem):
    rough_string = tostring(elem, 'utf-8')
    reparsed = xml.dom.minidom.parseString(rough_string)
    return reparsed.toprettyxml(indent="\t")


def iv(n, g):
    return g.vs.find(name=n).index


def removeDebugFlags(g):
    for v in g.vs["varSet"]:
        if "debugLevel" in v:
            dbg = v["debugLevel"]
            for k, v in dbg.items():
                dbg[k] = "false"

    g["showTopView"] = "false"
    g["showRobotView"] = "false"
    g["showTaggedImg"] = "false"
    g["benchmark"] = "false"


def serializeGraph(g, xmlfile, write=False):
    res = {}
    for attr in g.attributes():
        res[attr] = g[attr]

    pipeline = []
    for v in g.vs:
        n = copy.deepcopy(v["varSet"])
        n["name"] = v["name"]
        pipeline.append(n)

    res['pipeline'] = pipeline

    root = "LocalisationTest"
    xmlTree = ht.dict2et(res, root, listnames={
                         "pipeline": "filter", "dependencies": "dependency", "paramInts": "paramInt", "paramFloats": "ParamFloat"})
    xmlString = prettify(xmlTree)
    if not write:
        return xmlString
    else:
        text_file = open(xmlfile, "w")
        text_file.write("{0}".format(xmlString))
        text_file.close()


def addVertexFromDB(className, g):
    vertex = None
    for v in dbGraph.vs:
        if v["varSet"]["className"] == className:
            vertex = v
            break
    if vertex is not None:
        i = 1
        newName = "{0}{1}".format(className, i)
        while newName in g.vs["name"]:
            i += 1
            newName = "{0}{1}".format(className, i)
        g.add_vertex(newName)
        node = g.vs[iv(newName, g)]
        node["varSet"] = vertex["varSet"]
        node["name"] = newName
        return node
    else:
        return None


def removeVertex(name, g):
    g.delete_vertices([iv(name, g)])


def addEdge(name1, name2, g):
    def firstAvailableDep(n):
        for i in range(len(n["varSet"]["dependencies"])):
            if n["varSet"]["dependencies"][i] in ["?", None]:
                return i
        return None

    index = firstAvailableDep(g.vs[iv(name2, g)])
    if index is not None:
        g.vs[iv(name2, g)]["varSet"]["dependencies"][index] = name1
        g.add_edge(iv(name1, g), iv(name2, g))
        return True
    else:
        return False


def removeEdge(name1, name2, g):
    index = g.vs[iv(name2, g)]["varSet"]["dependencies"].index(name1)
    g.vs[iv(name2, g)]["varSet"]["dependencies"][index] = None
    g.delete_edges([(iv(name1, g), iv(name2, g))])


def buildDependences(g):
    for node in g.vs:
        dep = node["varSet"].get("dependencies")
        if not (dep is None):
            if "dependency" in dep:
                g.add_edge(iv(dep.get("dependency"), g), iv(node["name"], g))
            else:
                for d in dep:
                    if d != '?':
                        g.add_edge(iv(d, g), iv(node["name"], g))


def reduceData(pipeline):
    tags = ["paramInts", "paramFloats", "dependencies"]
    if type(pipeline.get("filter")).__name__ == "XmlDictConfig":
        pipeline["filter"] = [pipeline.get("filter")]

    for filt in pipeline.get("filter"):
        for t in tags:
            # print filt.get(t)
            if (type(filt.get(t)) == type({})):
                filt[t] = filt.get(t).values()[0]
            elif filt.get(t) is not None:
                # print filt.get(t)
                filt[t] = filt.get(t).values()
            else:
                filt[t] = []


def unserializeGraph(xmlfile, source=True):
    tree = ET.parse(xmlfile)
    root = tree.getroot()
    xmldict = XmlDictConfig(root)
    reduceData(xmldict.get("pipeline"))
    g = Graph(directed=True)
    for key, value in xmldict.items():
        if(key != "pipeline"):
            g[key] = value

    i = 0
    for f in xmldict.get("pipeline").get("filter"):
        if (f.get("name") == "source" and source) or (f.get("name") != "source"):
            g.add_vertex(name=f.get("name"))
            varSet = {}
            for key, value in f.iteritems():
                if key == "name":
                    continue
                varSet[key] = value

            g.vs[i]["varSet"] = varSet

            i += 1

    return g

filtersFile = "config/filtersBase.xml"
dbGraph = unserializeGraph(filtersFile)

#=========================================================================
if __name__ == "__main__":
    file = "remiLocalisation.xml"
    # file = sys.argv[1]
    g = unserializeGraph(file)
    # buildDependences(g)
    print serializeGraph(g, "truc.xml")
