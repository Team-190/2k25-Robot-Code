import networkx as nx
from networkx.drawing.nx_pydot import read_dot

# Load the graph from the DOT file
G = read_dot("/home/elliotscher/Documents/FRC/FRC_190/2k25-Robot-Code/src/main/java/frc/robot/subsystems/v3_Epsilon/superstructure/Superstructure.dot")

# Specify your start and end nodes
start = input("Enter the start node: ")
end = input("Enter the end node: ")

# Find the shortest path
try:
    path = nx.shortest_path(G, source=start, target=end)
    print("Shortest path:", path)
except nx.NetworkXNoPath:
    print(f"No path between {start} and {end}")
