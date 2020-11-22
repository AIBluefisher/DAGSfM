# sudo pip3 install igraph

import igraph
from PIL import Image

filePath = '/home/chenyu/Projects/distributed_colmap/build/gml.txt'
imagePath = '/home/chenyu/Projects/distributed_colmap/build/graph_image.png'
# label_list = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2]
label_file = '/home/chenyu/Projects/distributed_colmap/build/labels.txt'

def getValue(value):
    colorList = ['blue','green','purple','yellow','red','pink','orange','black','white','gray','brown','wheat', 
                'coral', 'alice blue', 'cyan', 'green yellow', 'light blue', 'hot pink', 'light green', 'gold']

    return colorList[int(value)]

def drawIGraph(filePath, imagePath):
    g = igraph.Graph.Read_GML(filePath)
    g.vs['label'] = ['']

    # image = igraph.plot(g, **visual_style)
    igraph.plot(g, imagePath)

def drawIGraphWithLabel(filePath, imagePath, label_list):
    g = igraph.Graph.Read_GML(filePath)
    g.vs['label'] = ['']

    visual_style = dict()
    visual_style['vertex_color'] = list(map(getValue, label_list))

    igraph.plot(g, imagePath, **visual_style)
    # image = igraph.plot(g, **visual_style)
    # image.save(imagePath)

def drawIGraphWithLabelFile(filePath, imagePath, label_file):
    f = open(label_file)
    s = f.read()
    label_list = s.split()
    # print(label_list)
    
    drawIGraphWithLabel(filePath, imagePath, label_list)

drawIGraphWithLabelFile(filePath, imagePath, label_file)