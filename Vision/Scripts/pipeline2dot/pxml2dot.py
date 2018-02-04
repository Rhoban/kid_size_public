#!/usr/bin/python
# -*- coding: utf-8 -*-

#
#  File Name	: 'pxml2dot.py'
#  Author	: Steve NGUYEN
#  Contact      : steve.nguyen.000@gmail.com
#  Created	: lundi, juin  6 2016
#  Revised	:
#  Version	:
#  Target MCU	:
#
#  This code is distributed under the GNU Public License
# 		which can be found at http://www.gnu.org/licenses/gpl.txt
#
#
#  Notes:	Quick and dirty hack
#           python pxml2dot.py XMLFILE > output.dot
#           dot -Tps2 output.dot -o outfile.pdf
#
# from pygraphviz import *
from graphviz import Digraph
from lxml import etree
import sys


tree = etree.parse(sys.argv[1])
roottag = tree.getroot().tag


PIPECOLORS = {'GOALSIMPLE': 'red',  'GOALMASK': 'blue',  'BALLBYROI': 'green'}

nodes = []
dependencies = {}
classname = {}
pipename = {}
colordep = {}


def get_pred(n, pred):
    for p in dependencies[n]:
        # pre = []
        # pre.append(p)
        if not n in pred:
            pred[n] = []
        pred[n].append(p)
        get_pred(p, pred)


for pipe in tree.xpath('/' + roottag + '/pipeline'):
    for fil in pipe:
        # print
        # print("NEW FILTER")
        name = ''
        pipe = ''
        deps = []
        classn = ''
        for child in fil:

            # print ("%s %s" % (child.tag, child.text))

            if child.tag == 'className':
                # name += child.text + ': '
                classn = child.text
            elif child.tag == 'name':
                name = child.text

            elif child.tag == 'pipename':
                pipe = child.text

            elif child.tag == 'dependencies':
                for dep in child:
                    # print("\tDEPS: %s" % (dep.text))
                    deps.append(dep.text)

        if pipe != 'debug':

            nodes.append(name)
            dependencies[name] = deps
            classname[name] = classn
            if pipe != '':
                pipename[name] = PIPECOLORS[pipe]
                # print pipe,  PIPECOLORS[pipe], pipename[name]
            else:
                pipename[name] = 'black'

# print nodes
# print dependencies
# print classname
# print pipename

dot = Digraph(comment=roottag)

for n in nodes:
    if pipename[n] != 'black':
        pred = {}
        get_pred(n, pred)
        colordep[n] = pred

for n in nodes:
    dot.node(n, classname[n] + ': ' + n, color=pipename[n])

    if n in colordep:
        c = pipename[n]
        if c != 'black':
            pp = colordep[n]
            # print pp
            for ppp in pp:
                # print ppp
                for pppp in set(pp[ppp]):
                    dot.edge(pppp, ppp, color=c)
    else:
        for dep in dependencies[n]:
            dot.edge(dep, n)

# for n in nodes:
#     print n


# print pred
# print
# print colordep

print dot
