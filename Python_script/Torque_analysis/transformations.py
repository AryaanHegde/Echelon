import math
import sympy as sp
import numpy as np
from angle_conversion import*

class TransformationsMixin:
    def __init__(self, joint_angles, lengths):
        # Numerical transformations
        self.t1, self.t2, self.t3, self.t4, self.t5, self.t6 = deg_to_rad(joint_angles)
        self.l1, self.l2, self.l3, self.l4, self.l5, self.l6 = lengths
        
        self.H_1b = np.array([
            [math.cos(self.t1), -math.sin(self.t1), 0, 0],
            [math.sin(self.t1), math.cos(self.t1), 0, 0],
            [0, 0, 1, self.l1],
            [0, 0, 0, 1]
        ])

        self.H_21 = np.array([
            [math.cos(self.t2), -math.sin(self.t2), 0, 0],
            [0, 0, 1, 0],
            [-math.sin(self.t2), -math.cos(self.t2), 0, 0],
            [0, 0, 0, 1]
        ])

        self.H_32 = np.array([
            [-math.sin(self.t3), -math.cos(self.t3), 0, self.l2],
            [math.cos(self.t3), -math.sin(self.t3), 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])

        self.H_43 = np.array([
            [math.cos(self.t4), -math.sin(self.t4), 0, 0],
            [0, 0, -1, -(self.l3 + self.l4)],
            [math.sin(self.t4), math.cos(self.t4), 0, 0],
            [0, 0, 0, 1]
        ])

        self.H_54 = np.array([
            [math.cos(self.t5), -math.sin(self.t5), 0, 0],
            [0, 0, 1, 0],
            [-math.sin(self.t5), -math.cos(self.t5), 0, 0],
            [0, 0, 0, 1]
        ])

        self.H_65 = np.array([
            [math.cos(self.t6), -math.sin(self.t6), 0, 0],
            [0, 0, -1, -self.l5],
            [math.sin(self.t6), math.cos(self.t6), 0, 0],
            [0, 0, 0, 1]
        ])

        self.H_e6 = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, self.l6],
            [0, 0, 0, 1]
        ])
        
        self.H_c11 = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, -self.l1/2],
            [0, 0, 0, 1]
        ])

        self.H_c22 = np.array([
            [1, 0, 0, self.l2/2],
            [0, 1, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])

        self.H_c33 = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, -self.l3/2],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])

        self.H_c44 = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, -self.l4/2],
            [0, 0, 0, 1]
        ])

        self.H_c55 = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, -self.l5/2],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])

        self.H_c66 = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, self.l6/2],
            [0, 0, 0, 1]
        ])

        self.j1b = np.array([0, 0, 0])
        self.j2b = np.array([0, 0, self.l1])
        self.j3b = (self.H_1b @ self.H_21 @ self.H_32)[:3, 3]
        self.j4b = (self.H_1b @ self.H_21 @ self.H_32 @ np.array([
            [math.cos(self.t4), -math.sin(self.t4), 0, 0],
            [0, 0, -1, -self.l3],
            [math.sin(self.t4), math.cos(self.t4), 0, 0],
            [0, 0, 0, 1]
        ]))[:3, 3]
        self.j5b = (self.H_1b @ self.H_21 @ self.H_32 @ self.H_43)[:3, 3]
        self.j6b = (self.H_1b @ self.H_21 @ self.H_32 @ self.H_43 @ self.H_54 @ self.H_65)[:3, 3]

        self.joint_transforms = [self.H_1b, self.H_21, self.H_32, self.H_43, self.H_54, self.H_65, self.H_e6]
        self.com_to_joint_transforms = [self.H_c11, self.H_c22, self.H_c33, self.H_c44, self.H_c55, self.H_c66]
        self.joint_position_vectors = [self.j1b, self.j2b, self.j3b, self.j4b, self.j5b, self.j6b]


        # Symbolic transformations
        self.ts1, self.ts2, self.ts3, self.ts4, self.ts5, self.ts6 = sp.symbols('t1 t2 t3 t4 t5 t6')
        self.ls1, self.ls2, self.ls3, self.ls4, self.ls5, self.ls6 = sp.symbols('l1 l2 l3 l4 l5 l6')

        self.Hs_1b = sp.Matrix([
            [sp.cos(self.ts1), -sp.sin(self.ts1), 0, 0],
            [sp.sin(self.ts1), sp.cos(self.ts1), 0, 0],
            [0, 0, 1, self.ls1],
            [0, 0, 0, 1]
        ])

        self.Hs_21 = sp.Matrix([
            [sp.cos(self.ts2), -sp.sin(self.ts2), 0, 0],
            [0, 0, 1, 0],
            [-sp.sin(self.ts2), -sp.cos(self.ts2), 0, 0],
            [0, 0, 0, 1]
        ])

        self.Hs_32 = sp.Matrix([
            [-sp.sin(self.ts3), -sp.cos(self.ts3), 0, self.ls2],
            [sp.cos(self.ts3), -sp.sin(self.ts3), 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])

        self.Hs_43 = sp.Matrix([
            [sp.cos(self.ts4), -sp.sin(self.ts4), 0, 0],
            [0, 0, -1, -(self.ls3 + self.ls4)],
            [sp.sin(self.ts4), sp.cos(self.ts4), 0, 0],
            [0, 0, 0, 1]
        ])

        self.Hs_54 = sp.Matrix([
            [sp.cos(self.ts5), -sp.sin(self.ts5), 0, 0],
            [0, 0, 1, 0],
            [-sp.sin(self.ts5), -sp.cos(self.ts5), 0, 0],
            [0, 0, 0, 1]
        ])

        self.Hs_65 = sp.Matrix([
            [sp.cos(self.ts6), -sp.sin(self.ts6), 0, 0],
            [0, 0, -1, -self.ls5],
            [sp.sin(self.ts6), sp.cos(self.ts6), 0, 0],
            [0, 0, 0, 1]
        ])

        self.Hs_e6 = sp.Matrix([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, self.ls6],
            [0, 0, 0, 1]
        ])

        self.Hs_c11 = sp.Matrix([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, -self.ls1/2],
            [0, 0, 0, 1]
        ])

        self.Hs_c22 = sp.Matrix([
            [1, 0, 0, self.ls2/2],
            [0, 1, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])

        self.Hs_c33 = sp.Matrix([
            [1, 0, 0, 0],
            [0, 1, 0, -self.ls3/2],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])

        self.Hs_c44 = sp.Matrix([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, -self.ls4/2],
            [0, 0, 0, 1]
        ])

        self.Hs_c55 = sp.Matrix([
            [1, 0, 0, 0],
            [0, 1, 0, -self.ls5/2],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])

        self.Hs_c66 = sp.Matrix([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, self.ls6/2],
            [0, 0, 0, 1]
        ])

        self.js1b = sp.Matrix([0, 0, 0])
        self.js2b = sp.Matrix([0, 0, self.ls1])
        self.js3b = (self.Hs_1b @ self.Hs_21 @ self.Hs_32)[:3, 3]
        self.js4b = (self.Hs_1b @ self.Hs_21 @ self.Hs_32 @ sp.Matrix([
            [sp.cos(self.ts4), -sp.sin(self.ts4), 0, 0],
            [0, 0, -1, -self.ls3],
            [sp.sin(self.ts4), sp.cos(self.ts4), 0, 0],
            [0, 0, 0, 1]
        ]))[:3, 3]
        self.js5b = (self.Hs_1b @ self.Hs_21 @ self.Hs_32 @ self.Hs_43)[:3, 3]
        self.js6b = (self.Hs_1b @ self.Hs_21 @ self.Hs_32 @ self.Hs_43 @ self.Hs_54 @ self.Hs_65)[:3, 3]

        self.s_joint_transforms = [self.Hs_1b, self.Hs_21, self.Hs_32, self.Hs_43, self.Hs_54, self.Hs_65, self.Hs_e6]
        self.s_com_to_joint_transforms = [self.Hs_c11, self.Hs_c22, self.Hs_c33, self.Hs_c44, self.Hs_c55, self.Hs_c66]
        self.s_joint_position_vectors = [self.js1b, self.js2b, self.js3b, self.js4b, self.js5b, self.js6b]
