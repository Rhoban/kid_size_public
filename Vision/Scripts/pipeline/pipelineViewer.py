#! /usr/bin/env python
import graphSerialization as gs
from generateBenchmark import generateBenchmark
from generateEmbedded import generateEmbedded
from Tkinter import *
from random import randint
from tkFileDialog import askopenfilename, asksaveasfile
from ScrolledText import *
import json
import ast

def dist(x1, y1, x2, y2):
    math.hypot(x2 - x1, y2 - y1)

def arrowFit(n1x1, n1x2, n2x1, n2x2):
    #TODO from center
    x1 = x2 = 0
    if (n1x1 > n2x2) :
        x1 = n1x1
        x2 = n2x2
    elif (n1x2 < n2x1) :
        x1 = n1x2
        x2 = n2x1
    else :
        x1 = (n1x2 - n1x1)/2 + n1x1
        x2 = (n2x2 - n2x1)/2 + n2x1
    return (x1, x2)

def primTag(n):
    return "{0}_primary".format(n)

class Node:
    defaultColor = "white"
    height = 40
    width = 65
    r = 3
    def __init__(self, x, y, size, canv, tag):
        self.tag = tag
        self.c = canv
        self.rect = self.c.create_rectangle(x, y, x+self.width, y+self.height, fill=self.defaultColor, tags=tag)
        self.c.addtag_withtag(primTag(self.tag), self.rect)
        self.text = self.c.create_text(x + self.width/2, y + self.height/2, width=self.width, text=tag, tags=tag)
        self.ovals = []
        for i in range(size):
            ny = (i+1)*(self.height/(size+1))
            ntag = "{0}_{1}".format(tag, i)
            oval = self.c.create_oval(x-self.r, y+ny-self.r, x+self.r, y+ny+self.r, fill="black", tags=ntag)
            self.c.addtag_withtag(self.tag, oval)
            self.ovals.append(oval)

    def delete(self):
        self.c.delete(self.rect)
        self.c.delete(self.text)
        for i in self.ovals:
            self.c.delete(i)



class Edge:
    defaultColor = "black"
    def __init__(self, p0, p1, canv, n1, n2, tag):
        self.tag = tag
        self.c = canv
        self.n1 = n1
        self.n2 = n2
        self.arrow = self.c.create_line(p0[0], p0[1], p1[0], p1[1], activewidth=2, arrow="last", fill=self.defaultColor, tags=tag)
        self.c.addtag_withtag(primTag(self.tag), self.arrow)

    def delete(self):
        self.c.delete(self.arrow)

class Application:
    def __init__(self, file=None):
        self.root=Tk()
        self.root.title("Pipeline Viewer")
        self.file = file
        self.dbGraph = gs.dbGraph
        self.WIDTH = 1000
        self.HEIGHT = 800
        #gs.plot(self.graph, bbox=(self.WIDTH - 100, self.HEIGHT-100))
        self.root.geometry("{0}x{1}".format(self.WIDTH, self.HEIGHT))
        self.root.columnconfigure(0, weight=10)
        self.root.columnconfigure(1, weight=1)
        #self.root.rowconfigure(0, weight=10)
        self.root.rowconfigure(1, weight=10)
        #self.root.rowconfigure(2, weight=10)

        self.nodes = []
        self.edges = []
        self.selColor = "green"
        self.currSel = None
        self.isCtrlDown = False
        self.drawingEdge = False

        self.c = Canvas(self.root, bg="white")
        self.c.grid(row=0, column=0, rowspan=5, sticky=N+S+W+E)

        self.stateBar = Label(self.root, text="", bd=1, relief=SUNKEN, anchor=W)
        self.stateBar.grid(row=4, columnspan=3, sticky=E+W)



        self.list = Listbox(self.root, width=30)
        self.sbar = Scrollbar(self.root)
        self.sbar.config(command=self.list.yview)
        self.list.config(yscrollcommand=self.sbar.set)
        self.list.grid(row=1, column=1, sticky=N+S+E+W)
        self.sbar.grid(row=1, column=2, sticky=N+S)
        for i in range(len(self.dbGraph.vs)):
            self.list.insert(i, self.dbGraph.vs[i]["varSet"]["className"])

        self.addButton = Button(self.root, text="Add", command=self.addToLayout)
        self.addButton.grid(row=0, column=1, columnspan=2, sticky=E+W)

        self.addButton = Button(self.root, text="Save", command=self.saveAttributes)
        self.addButton.grid(row=2, column=1, columnspan=2, sticky=E+W)
        self.scrolledText = ScrolledText(self.root, width = 30, undo=True, setgrid=True, wrap ='word')
        self.scrolledText.grid(row=3, column=1, columnspan=2, sticky=N+S+W+E)

        self.scrolledText.insert("insert","")



        self.dummyEdge = Edge((-1, -1), (-1, -1), self.c, None, None, "dummyEdge")
        self.c.bind("<Button-1>", self.mouseDown)
        self.c.bind("<Button1-Motion>", self.mouseMove)
        self.c.bind("<Button1-ButtonRelease>", self.mouseUp)
        self.root.bind("<KeyPress-Control_L>", self.controlDown)
        self.root.bind("<KeyRelease-Control_L>", self.controlUp)
        self.root.bind("<Delete>", self.deleteSelection)

        self.list.bind('<<ListboxSelect>>', self.deselect)

        if self.file is not None:
            self.openLayout(dialog = False)
        else:
            self.newLayout()

        menubar = Menu(self.root)
        filemenu = Menu(menubar, tearoff=0)
        filemenu.add_command(label="New", command = self.newLayout)
        filemenu.add_command(label="Open", command = self.openLayout)
        filemenu.add_command(label="Save", command = self.saveLayout)
        filemenu.add_separator()
        filemenu.add_command(label="Exit", command=self.root.quit)
        menubar.add_cascade(label="File", menu=filemenu)
        editmenu = Menu(menubar, tearoff=0)
        editmenu.add_command(label="Reorder")
        menubar.add_cascade(label="Edit", menu=editmenu)
        self.root.config(menu=menubar)

        self.root.after(10, self.drawLoop)
        self.root.mainloop()

    def newLayout(self):
        for n in self.nodes:
            n.delete()
        for e in self.edges:
            e.delete()
        self.nodes, self.edges = [], []
        self.currSel = None
        self.isCtrlDown = False
        self.drawingEdge = False
        self.graph = gs.Graph()
        self.graph.vs["name"] =""

    def openLayout(self, dialog=True):
        if dialog:
            self.file = askopenfilename(filetypes=(("XML files", "*.xml"),
                                           ("All files", "*.*") ))
        self.newLayout()
        self.graph = gs.unserializeGraph(self.file)
        gs.buildDependences(self.graph)
        self.topologicalGrid()
        #gs.plot(self.graph, bbox=(self.WIDTH - 100, self.HEIGHT-100))
        self.createNodesFromGraph()
        self.createEdgesFromGraph()


    def saveLayout(self):
        f = asksaveasfile(mode='w', defaultextension=".xml", filetypes=(("XML files", "*.xml"),
                                       ("All files", "*.*") ))
        if f is None:
            return
        f.write(gs.serializeGraph(self.graph, f.name))
        f.close()

        name = f.name
        name = name.replace(".xml", "")
        generateBenchmark(f.name, "{0}Benchmark.xml".format(name))
        generateEmbedded(f.name, "{0}Embedded.xml".format(name))


    def addToLayout(self):
        self.addNodeFromDB(self.list.get(ACTIVE))

    def saveAttributes(self):
        if self.currSel in self.nodes:
            attr = ast.literal_eval(self.scrolledText.get(1.0, END))
            self.graph.vs[gs.iv(self.currSel.tag, self.graph)]["varSet"] = attr



    def topologicalGrid(self):
        positions = self.graph.topological_sorting()
        width = 85
        height = (self.HEIGHT-250)/(max([n.degree() for n in self.graph.vs])+1)
        height2 = (self.HEIGHT)/len(self.graph.vs)
        beg = filter(lambda x: self.graph.vs[x].indegree() == 0, positions)
        self.graph.vs["x"] = 5
        self.graph.vs["y"] = 5
        i = 0
        for p in beg:
            self.graph.vs[p]["y"] += i * height2
            i += 1
        for p in positions:
            node = self.graph.vs[p]
            j = 0
            order = sorted(node.successors(), key = lambda x : len(filter(lambda n: isinstance(n, int), x.shortest_paths()[0])), reverse=True)
            for succ in order:
                succ["x"] = node["x"] + width
                succ["y"] = node["y"] + j * height 

                j += 1
        


    def tagNode(self, n1, n2):
        return "{0}_to_{1}".format(n1, n2)

    def findNode(self, name):
        for n in self.nodes:
            if name == n.tag:
                return n
        return None

    def findEdge(self, n1, n2=None):
        if n2 is None :
            tag = n1
        else :
            tag = self.tagNode(n1, n2)
        for e in self.edges:
            if tag == e.tag:
                return e
        return None

    def addNodeFromDB(self, n):
        node = gs.addVertexFromDB(n, self.graph)
        if node is not None:
            node["x"] = self.WIDTH - Node.width
            node["y"] = 0
            self.nodes.append(Node(node["x"], node["y"], len(node["varSet"]["dependencies"]), self.c, node["name"]))


    def addEdge(self, n1, n2):
        if (self.findEdge(n1, n2) is None) and (n1 != n2):
            if gs.addEdge(n1, n2, self.graph):
                node1 = self.findNode(n1)
                node2 = self.findNode(n2)
                self.edges.append(Edge(self.c.coords(node1.rect), self.c.coords(node2.rect), self.c, node1, node2, self.tagNode(n1, n2)))


    def createNodesFromGraph(self):
        for n in self.graph.vs:
            self.nodes.append(Node(n["x"], n["y"], len(n["varSet"]["dependencies"]), self.c, n["name"]))
        for n in self.nodes:
            coords = self.c.coords(n.rect)
            while len(self.c.find_overlapping(coords[0], coords[1], coords[2], coords[3])) == 2 + len(n.ovals) and coords[1] > 0:
                self.c.move(n.tag, 0, -1)
                coords = self.c.coords(n.rect)
            
            self.c.move(n.tag, 0, 1.5*Node.height)

    def createEdgesFromGraph(self):
        for e in self.graph.get_edgelist():
            n1 = self.graph.vs[e[0]]["name"]
            n2 = self.graph.vs[e[1]]["name"]
            node1 = self.findNode(n1)
            node2 = self.findNode(n2)
            self.edges.append(Edge(self.c.coords(node1.rect), self.c.coords(node2.rect), self.c, node1, node2, self.tagNode(n1, n2)))

    def drawLoop(self):
        for e in self.edges:
            n1 = self.c.coords(e.n1.rect)
            index = self.graph.vs[gs.iv(e.n2.tag, self.graph)]["varSet"]["dependencies"].index(e.n1.tag)
            n2 = self.c.coords(e.n2.ovals[index])

            xs = arrowFit(n1[0], n1[2], n2[0], n2[2])
            ys = arrowFit(n1[1], n1[3], n2[1], n2[3])
            self.c.coords(e.tag, xs[0], ys[0], xs[1], ys[1])

        self.root.after(10, self.drawLoop)

    def removeEdge(self, n1, n2):
        edge = self.findEdge(n1, n2)
        edge.delete()
        gs.removeEdge(n1, n2, self.graph)
        self.edges.remove(edge)

    def removeNode(self, n):
        node = self.findNode(n)
        for succ in self.graph.vs[gs.iv(n, self.graph)].successors():
            self.removeEdge(node.tag, succ["name"])

        for pred in self.graph.vs[gs.iv(n, self.graph)].predecessors():
            self.removeEdge(pred["name"], node.tag)

        gs.removeVertex(node.tag, self.graph)
        node.delete()
        self.nodes.remove(node)


    def inRect(self, x, y, rect):
        if(x < rect[0]):
            return False
        if(y < rect[1]):
            return False
        if(x > rect[2]):
            return False
        if(y > rect[3]):
            return False
        return True

    def deselect(self, event):
        if self.currSel is not None:
            self.c.itemconfig(primTag(self.currSel.tag), fill=self.currSel.defaultColor)
            self.c.itemconfig(primTag(self.currSel.tag), width=1)
            self.currSel=None

    def handleSelection(self, event):
        self.list.selection_clear(0, END)
        dstObject = self.c.find_overlapping(event.x, event.y, event.x+1, event.y+1)
        objs = self.c.gettags(dstObject)
        self.deselect(event)
        self.scrolledText.delete("1.0",END)
        self.scrolledText.insert("insert","")
        if(len(dstObject)):
            objs = self.c.gettags(dstObject[0])
            s = self.findNode(objs[0])
            if s is None:
                s = self.findEdge(objs[0])
            if s is not None:
                self.currSel = s
                self.c.itemconfig(primTag(self.currSel.tag), fill=self.selColor)
                self.c.itemconfig(primTag(self.currSel.tag), width=2)

    def mouseDown(self, event):
        self.handleSelection(event)
        self.x1, self.y1 = event.x, event.y
        if self.currSel in self.nodes:
            self.scrolledText.insert("insert", json.dumps(self.graph.vs[gs.iv(self.currSel.tag, self.graph)]["varSet"], indent=4).replace("null", "None"))

            if self.isCtrlDown :
                self.drawingEdge = True
                self.c.coords("dummyEdge", self.x1, self.y1, self.x1, self.y1)

    def mouseMove(self, event):
        x2, y2 = event.x, event.y
        if not self.drawingEdge :
            dx, dy = x2 -self.x1, y2 -self.y1
            if self.currSel is not None:
                self.c.move(self.currSel.tag, dx, dy)
                self.x1, self.y1 = x2, y2
        else :
            coords = self.c.coords("dummyEdge")
            self.c.coords(self.dummyEdge.arrow, coords[0], coords[1], x2, y2)
            self.c.tag_raise(self.dummyEdge.arrow)

    def mouseUp(self, event):
        if not self.drawingEdge :
            if self.currSel is not None:
                self.c.itemconfig(primTag(self.currSel.tag), width=1)
        else :
            dstObject = self.c.find_overlapping(event.x, event.y, event.x+1, event.y+1)[0]
            dstTag = self.c.gettags(dstObject)[0]
            if (dstTag in [n.tag for n in self.nodes]) and (self.inRect(event.x, event.y, self.c.coords(dstTag))):
                self.addEdge(self.currSel.tag, dstTag)

            self.drawingEdge = False
            self.c.coords(self.dummyEdge.arrow, -1, -1, -1, -1)

    def deleteSelection(self, event):
        if self.currSel is not None:
            if self.currSel in self.nodes:
                self.removeNode(self.currSel.tag)
            elif self.currSel in self.edges:
                self.removeEdge(self.currSel.n1.tag, self.currSel.n2.tag)

        self.currSel = None

    def controlDown(self, event):
        self.isCtrlDown = True

    def controlUp(self, event):
        self.isCtrlDown = False




if __name__ == "__main__":
    file = None
    if((len(sys.argv) == 2)):
        file = sys.argv[1]
    app = Application(file)
