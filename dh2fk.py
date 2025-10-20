from readline import redisplay
from numpy import disp
import sympy as sp

# Use this in a Jupyter Notebook or similar for nicely formatted math output
sp.init_printing(use_unicode=True)

# -------------------------------------------------------------------
# Part 1: Symbolic Derivation
# -------------------------------------------------------------------

# 1. Define the symbolic variables
l1, l3, d3, l5, theta5, l7 = sp.symbols('l1 l3 d3 l5 theta5 l7')

# 2. Define the DH table using the symbolic variables
# [theta, d, a/r, alpha] in radians
dh_table = [
    [0,      0,     -l1,    0],
    [sp.pi/2, 0,     0,      0],
    [0,      l3+d3, 0,      0],
    [0,      0,     0,      sp.pi/2],
    [0,      l5,    0,      0],
    [theta5, 0,     0,     -sp.pi/2],
    [-sp.pi/2, l7,    0,      0]
]

def create_symbolic_transform(dh_row):
    """Creates a 4x4 symbolic transformation matrix from a DH table row."""
    theta, d, a, alpha = dh_row
    
    return sp.Matrix([
        [sp.cos(theta), -sp.sin(theta)*sp.cos(alpha),  sp.sin(theta)*sp.sin(alpha), a*sp.cos(theta)],
        [sp.sin(theta),  sp.cos(theta)*sp.cos(alpha), -sp.cos(theta)*sp.sin(alpha), a*sp.sin(theta)],
        [0,              sp.sin(alpha),                sp.cos(alpha),                d],
        [0,              0,                            0,                            1]
    ])

# 3. Initialize the final transformation matrix as a 4x4 identity matrix
T_final_symbolic = sp.eye(4)

# 4. Loop through the DH table and multiply the matrices
for row in dh_table:
    T_final_symbolic = T_final_symbolic * create_symbolic_transform(row)

# 5. Simplify the final expression for a cleaner result
T_final_symbolic = sp.simplify(T_final_symbolic)

# 6. Display the final symbolic transformation matrix row by row
print("Final Symbolic Transformation Matrix:")
for i in range(T_final_symbolic.rows):
    print(T_final_symbolic.row(i))
