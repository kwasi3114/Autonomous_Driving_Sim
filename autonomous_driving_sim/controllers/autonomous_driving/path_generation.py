import networkx as nx
import numpy as np
from scipy.interpolate import splprep, splev

x = [200,180,160,140,120,100,80,60,50,40,38,38,38,38,38,38,38,38,
     40,45,55,70,85,105,120,140,160,
     180,200,230,260,290,320,350,
     365,380,400,425,440,450,456,458,
     458,458,458,458,458,458,458,
     453,445,430,410,390,370,350,
     320,300,270,240]
y = [190,192.5,192.5,195,197.5,205,220,240,260,280,300,320,350,380,410,440,470,500,
     520,540,560,580,590,600,607,615,615,
     615,615,615,615,615,615,615,
     610,605,595,580,560,540,520,500,
     470,440,410,380,350,320,300,
     280,260,238,215,205,197.5,195,
     193,193,193,193]

#x = [200,180,160,140,120,100,80,60,50,40]
#y = [190,192.5,192.5,195,197.5,205,210,225,240,260]

def setCoords():
    # Create directed graph
    G = nx.DiGraph()
    
    # Add nodes with position attributes
    for i, (xi, yi) in enumerate(zip(x, y)):
        G.add_node(i, pos=(xi, yi))
    
    # Add directed edges from each point to the next (do NOT wrap around)
    for i in range(len(x) - 1):
        G.add_edge(i, i + 1)
    
    # Optional: Add weights based on Euclidean distance
    for i, j in G.edges():
        xi, yi = G.nodes[i]['pos']
        xj, yj = G.nodes[j]['pos']
        dist = ((xj - xi)**2 + (yj - yi)**2)**0.5
        G[i][j]['weight'] = dist
        
    return G
    

def createPathCurve(G):
    # Suppose we want to go from node 0 to node 59 (the last one)
    source = 0
    target = len(G.nodes) - 1
    
    # Compute shortest path using distance weights
    path_nodes = nx.shortest_path(G, source=source, target=target, weight='weight')
    
    # Extract coordinates of nodes in the path
    path_coords = [G.nodes[n]['pos'] for n in path_nodes]
    
    # Convert path to arrays
    path_coords = np.array(path_coords)
    x, y = path_coords[:, 0], path_coords[:, 1]
    
    # Fit a cubic spline (s=0 → exact fit)
    tck, u = splprep([x, y], s=5)  # try higher s for smoother curves
    
    # Interpolate 200 evenly spaced points along the spline
    u_fine = np.linspace(0, 1, 200)
    x_smooth, y_smooth = splev(u_fine, tck)
    
    return x_smooth, y_smooth

def pixelToWebots(x_pix, y_pix):
    p0 = (x[0], y[0])       # pixel coordinates of node 0
    p1 = (x[42], y[42])     # pixel coordinates of node 42
    
    w0 = (-45.000000090289845, 45.88000000000215)   # Webots coords
    w1 = (47.772702946422186, -56.5538993781892)
    
    x_img = np.array([p0[0], p1[0]])
    y_img = np.array([p0[1], p1[1]])
    
    x_world = np.array([w0[0], w1[0]])
    z_world = np.array([w0[1], w1[1]])
    
    a_x, b_x = np.polyfit(x_img, x_world, 1)
    a_z, b_z = np.polyfit(y_img, z_world, 1)

    Xw = a_x * x_pix + b_x
    Zw = a_z * y_pix + b_z
    return Xw, Zw
    
def pathGeneration():
    #G = setCoords()
    #x_curve, y_curve = createPathCurve(G)
    #x_path, y_path = pixelToWebots(x_curve, y_curve)
    x_path = [-45,-57,-72.6,-86.5,-95.6,-105,-108,-108,-108,-108,-108,
    -107,-101,-93,-78.4,-59.8,-46.1,-36.4,-19.9,-7.26,9.01,17.8,28.9,37.3,
    44.3,46.1,47.1]
    
    y_path = [45.9,46.3,45.8,40.8,33,18.8,0.545,-16.3,-35.5,-55.5,-63.8,
    -77.2,-89.5,-98.3,-106,-108,-109,-109,-109,-108,-108,-106,
    -101,-94.1,-83.4,-73.7,-62.5]
    
    print("Path Generated")
    return x_path, y_path