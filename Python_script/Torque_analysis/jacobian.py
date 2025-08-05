import numpy as np
import sympy as sp

class JacobianMixin:
    def calculate_numerical_link_jacobian(self, link):
        joint_axes = []
        p_icomb = []

        for i in range(6):
            H_ib = np.array([
                [1, 0, 0, 0],
                [0, 1, 0, 0],
                [0, 0, 1, 0],
                [0, 0, 0, 1]
            ])

            for j in range(i+1):
                H_ib = H_ib @ self.T.joint_transforms[j]

            z_ib = H_ib[:3, 2]

            if link == (i+1):
                p_icomb = (H_ib @ self.T.com_to_joint_transforms[i])[:3, 3]

            joint_axes.append(z_ib)

        linear_jacobian_matrix = []
        angular_jacobian_matrix = []
        zeros = []
        for joint_iter in range(link):
            linear_jacobian_matrix.append(np.cross(joint_axes[joint_iter], p_icomb - self.T.joint_position_vectors[joint_iter]))
            angular_jacobian_matrix.append(joint_axes[joint_iter])

        for _ in range(6 - link):
            zeros.append([0, 0, 0])

        zeros = np.transpose(np.array(zeros))

        linear_jacobian_matrix = np.array(linear_jacobian_matrix)
        linear_jacobian_matrix = np.transpose(linear_jacobian_matrix)

        angular_jacobian_matrix = np.array(angular_jacobian_matrix)
        angular_jacobian_matrix = np.transpose(angular_jacobian_matrix)

        if (link != 6):
            linear_jacobian_matrix = np.concatenate((linear_jacobian_matrix, zeros), axis=1)
            angular_jacobian_matrix = np.concatenate((angular_jacobian_matrix, zeros), axis=1)
        
        return [linear_jacobian_matrix, angular_jacobian_matrix]


    def calculate_symbolic_link_jacobian(self, link):
        joint_axes = []
        p_icomb = []

        for i in range(6):
            H_ib = sp.Matrix([
                [1, 0, 0, 0],
                [0, 1, 0, 0],
                [0, 0, 1, 0],
                [0, 0, 0, 1]
            ])

            for j in range(i+1):
                H_ib = H_ib @ self.T.s_joint_transforms[j]

            z_ib = H_ib[:3, 2]

            if link == (i+1):
                p_icomb = (H_ib @ self.T.s_com_to_joint_transforms[i])[:3, 3]

            joint_axes.append(z_ib)

        linear_jacobian_matrix = []
        angular_jacobian_matrix = []
        zeros = []

        for joint_iter in range(link):
            S_ja = sp.Matrix([
                [0, -joint_axes[joint_iter][2], joint_axes[joint_iter][1]],
                [joint_axes[joint_iter][2], 0, -joint_axes[joint_iter][0]],
                [-joint_axes[joint_iter][1], joint_axes[joint_iter][0], 0]
            ])
            
            linear_jacobian_matrix.append(S_ja @ (p_icomb - self.T.s_joint_position_vectors[joint_iter]))
            angular_jacobian_matrix.append(joint_axes[joint_iter])

        for _ in range(6 - link):
            zeros.append([0, 0, 0])

        zeros = sp.transpose(sp.Matrix(zeros))
        linear_jacobian_matrix = sp.Matrix.hstack(*linear_jacobian_matrix)
        angular_jacobian_matrix = sp.Matrix.hstack(*angular_jacobian_matrix)
        if (link != 6):
            linear_jacobian_matrix = linear_jacobian_matrix.row_join(zeros)
            angular_jacobian_matrix = angular_jacobian_matrix.row_join(zeros)

        return [linear_jacobian_matrix, angular_jacobian_matrix]

