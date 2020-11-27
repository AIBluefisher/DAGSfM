# sudo pip3 install igraph

import igraph
import sys
from PIL import Image

graph_dir = sys.argv[1]

file_path = graph_dir + '/gml.txt'
label_file = graph_dir + '/labels.txt'
image_path = graph_dir + '/graph_image.png'

def getValue(value):
    colorList = ['blue','green','purple','yellow','red','pink','orange','black','white','gray','brown','wheat', 
                'coral', 'alice blue', 'cyan', 'green yellow', 'light blue', 'hot pink', 'light green', 'gold']

    return colorList[int(value)]

def drawIGraph(file_path, image_path):
    g = igraph.Graph.Read_GML(file_path)
    g.vs['label'] = ['']

    igraph.plot(g, image_path)

def drawIGraphWithLabel(file_path, image_path, label_list):
    g = igraph.Graph.Read_GML(file_path)
    g.vs['label'] = ['']

    visual_style = dict()
    visual_style['vertex_color'] = list(map(getValue, label_list))

    igraph.plot(g, image_path, **visual_style)
    # image = igraph.plot(g, **visual_style)
    # image.save(image_path)

def drawIGraphWithLabelFile(file_path, image_path, label_file):
    f = open(label_file)
    s = f.read()
    label_list = s.split()
    # print(label_list)
    
    drawIGraphWithLabel(file_path, image_path, label_list)

drawIGraphWithLabelFile(file_path, image_path, label_file)