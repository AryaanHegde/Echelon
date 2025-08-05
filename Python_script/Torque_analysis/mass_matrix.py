import numpy as np
import sympy as sp

class MassMatrixMixin:
    def set_numerical_mass_matrix(self):
        mass_matrix = np.zeros((6,6))
        for link in range(1, 7):
            inertia_tensor_com = None
            if link == 1:
                inertia_tensor_com = np.array([
                    [110.858, 0, 0],
                    [0, 110.858, 0],
                    [0, 0, 157.076]
                ]) * 0.00001828
            else:
                inertia_tensor_com = np.array([
                    [10917.841, 0, 0],
                    [0, 1125.346, 66.205],
                    [0, 66.205, 11448.422]
                ]) * 0.00001828

            H_ib = np.array([
                [1, 0, 0, 0],
                [0, 1, 0, 0],
                [0, 0, 1, 0],
                [0, 0, 0, 1]
            ])

            for i in range(link):
                H_ib = H_ib @ self.T.joint_transforms[i]
            
            H_icomb = H_ib @ self.T.com_to_joint_transforms[link-1]
            R_icomb = H_icomb[:3, :3]
            link_mass = self.masses[link-1]

            ljm, ajm = self.calculate_numerical_link_jacobian(link)
            inertia_tensor_base = R_icomb @ inertia_tensor_com @ np.transpose(R_icomb)

            mass_matrix += (link_mass * (np.transpose(ljm) @ ljm)) + (np.transpose(ajm) @ inertia_tensor_base @ ajm)
        
        self.num_mass_matrix = mass_matrix
    
    def set_symbolic_mass_matrix(self):
        mass_matrix = sp.zeros(6)
        for link in range(1, 7):
            inertia_tensor_com = None
            if link == 1:
                inertia_tensor_com = sp.Matrix([
                    [110.858, 0, 0],
                    [0, 110.858, 0],
                    [0, 0, 157.076]
                ]) * 0.00001828
            else:
                inertia_tensor_com = sp.Matrix([
                    [10917.841, 0, 0],
                    [0, 1125.346, 66.205],
                    [0, 66.205, 11448.422]
                ]) * 0.00001828

            H_ib = sp.Matrix([
                [1, 0, 0, 0],
                [0, 1, 0, 0],
                [0, 0, 1, 0],
                [0, 0, 0, 1]
            ])

            for i in range(link):
                H_ib = H_ib @ self.T.s_joint_transforms[i]
            
            H_icomb = H_ib @ self.T.s_com_to_joint_transforms[link - 1]
            R_icomb = H_icomb[:3, :3]
            link_mass = self.masses[link - 1]

            ljm, ajm = self.calculate_symbolic_link_jacobian(link)
            inertia_tensor_base = R_icomb @ inertia_tensor_com @ sp.transpose(R_icomb)

            mass_matrix += (link_mass * (sp.transpose(ljm) @ ljm)) + (sp.transpose(ajm) @ inertia_tensor_base @ ajm)
        
        self.sym_mass_matrix = mass_matrix

