import math
import numpy as np

class ForwardKinematics:
    def __init__(self, lengths, c_space):
        self.l1, self.l2, self.l3, self.l4, self.l5, self.l6 = lengths
        self.c_space = c_space
        self.t1, self.t2, self.t3, self.t4, self.t5, self.t6 = self.c_space

    def set_matrices(self):
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

    def solve(self):
        self.set_matrices()
        H_eb = self.H_1b @ self.H_21 @ self.H_32 @ self.H_43 @ self.H_54 @ self.H_65 @ self.H_e6
        r_eb = H_eb[:3, :3]
        pos_vector = H_eb[:3, 3]

        # Adjusted rotation to match RPY frame convention
        r_matrix = r_eb

        # RPY decomposition
        alpha = math.atan2(-r_matrix[1, 2], r_matrix[2, 2])
        beta = math.asin(r_matrix[0, 2])
        gamma = math.atan2(-r_matrix[0, 1], r_matrix[0, 0])

        orient_vector = np.array([alpha, beta, gamma])
        self.op_space = np.concatenate((pos_vector, orient_vector))
        return self.op_space


class InverseKinematics:
    def __init__(self, lengths, op_space):
        self.l1, self.l2, self.l3, self.l4, self.l5, self.l6 = lengths
        self.op_space = op_space
        self.x, self.y, self.z, self.alpha, self.beta, self.gamma = self.op_space
    
    def solve(self):
        pos_vector_ee = self.op_space[:3]
        alpha, beta, gamma = self.op_space[3:]

        # Construct rotation matrix from RPY
        R_x = np.array([
            [1, 0, 0],
            [0, math.cos(alpha), -math.sin(alpha)],
            [0, math.sin(alpha), math.cos(alpha)]
        ])

        R_y = np.array([
            [math.cos(beta), 0, math.sin(beta)],
            [0, 1, 0],
            [-math.sin(beta), 0, math.cos(beta)]
        ])

        R_z = np.array([
            [math.cos(gamma), -math.sin(gamma), 0],
            [math.sin(gamma), math.cos(gamma), 0],
            [0, 0, 1]
        ])

        R_eb = R_z @ R_y @ R_x
        # print(f"R_eb = {R_eb}")
        z_eb = R_eb[:, 2]  # Z-axis of end-effector in base frame
        # print(f"z_eb = {z_eb}")
        wc_to_ee = (self.l5 + self.l6) * z_eb
        # print(f"wc_to_ee = {wc_to_ee}")
        # print(f"pos_vector_ee = {pos_vector_ee}")
        pos_vector_wc = pos_vector_ee - wc_to_ee
        # print(f"pos_vector_wc = {pos_vector_wc}")
        p_x, p_y, p_z = pos_vector_wc
        self.t1 = math.atan2(p_y,p_x) + np.pi
        # print(f"self.t1 = {self.t1}")
        
        r_squared = p_x**2 + p_y**2
        s = p_z - self.l1
        # print(f"r^2 = {r_squared}")
        # print(f"s = {s}")

        D = (r_squared + s**2 - self.l2**2 - (self.l3 + self.l4)**2)/(2*self.l2*(self.l3 + self.l4))
        D = round(D, 6)
        D = max(min(D, 1.0), -1.0)
        self.t3 = math.asin(math.sqrt(1 - D**2))
        self.t2 = math.atan2(math.sqrt(r_squared), s) - math.atan2(self.l2 + (self.l3 + self.l4) * math.cos(self.t3), (self.l3 + self.l4) * math.sin(self.t3))
        R_3b = np.array([
            [-math.sin(self.t2 + self.t3)*math.cos(self.t1), -math.cos(self.t1)*math.cos(self.t1 + self.t2), -math.sin(self.t1)],
            [-math.sin(self.t1)*math.sin(self.t2 + self.t3), -math.sin(self.t1)*math.cos(self.t2 + self.t3), math.cos(self.t1)],
            [-math.cos(self.t2 + self.t3), math.sin(self.t2 + self.t3), 0]
        ])

        R_b3 = np.transpose(R_3b)
        R_e3 = R_b3 @ R_eb
        # print(f"R_e3[2,2]/R_e3[0,2] = {R_e3[2,2]/R_e3[0,2]}")
        print(f"-R_e3[1,2] = {-R_e3[1,2]}")
        self.t5 = math.acos(-R_e3[1,2])
        self.t6 = math.atan2(-R_e3[1,1],R_e3[1,0])
        self.t4 = math.atan2(R_e3[2,2],R_e3[0,2])

        self.c_space = np.array([self.t1, self.t2, self.t3, self.t4, self.t5, self.t6])
        return self.c_space


