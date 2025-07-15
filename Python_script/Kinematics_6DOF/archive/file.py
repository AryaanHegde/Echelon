import math
import sympy as sp
import numpy as np

l1, l2, l3, l4, l5, l6 = sp.symbols('L_1 L_2 L_3 L_4 L_5 L_6')
t1, t2, t3, t4, t5, t6 = sp.symbols('theta_1 theta_2 theta_3 theta_4 theta_5 theta_6')

H_43 = sp.Matrix([
    [sp.cos(t4), -sp.sin(t4), 0, 0],
    [0, 0, -1, -(l3 + l4)],
    [sp.sin(t4), sp.cos(t4), 0, 0],
    [0, 0, 0, 1]
])

H_54 = sp.Matrix([
    [sp.cos(t5), -sp.sin(t5), 0, 0],
    [0, 0, 1, 0],
    [-sp.sin(t5), -sp.cos(t5), 0, 0],
    [0, 0, 0, 1]
])

H_65 = sp.Matrix([
    [sp.cos(t6), -sp.sin(t6), 0, 0],
    [0, 0, -1, -l5],
    [sp.sin(t6), sp.cos(t6), 0, 0],
    [0, 0, 0, 1]
])

H_e6 = sp.Matrix([
    [1, 0, 0, 0],
    [0, 1, 0, 0],
    [0, 0, 1, l6],
    [0, 0, 0, 1]
])

R_e3 = H_43 @ H_54 @ H_65 @ H_e6
sp.pprint(R_e3)