import networkx as nx
from networkx.drawing.nx_pydot import read_dot
import os

# Load the graph from the DOT file
dot_file_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "../src/main/java/frc/robot/subsystems/v3_Poot/superstructure/Superstructure.dot"))
G = read_dot(dot_file_path)

# Specify your start and end nodes
start = input("Enter the start node: ")
end = input("Enter the end node: ")

# Find the shortest path
try:
    path = nx.shortest_path(G, source=start, target=end)
    print("Shortest path:", path)
except nx.NetworkXNoPath:
    print(f"No path between {start} and {end}")
