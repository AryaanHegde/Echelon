import sympy as sp

class TorqueMixin:
    def set_torques(self):
        torques = [0, 0, 0, 0, 0, 0]
        sym_angles = [self.T.ts1, self.T.ts2, self.T.ts3, self.T.ts4, self.T.ts5, self.T.ts6]
        angles = [self.T.t1, self.T.t2, self.T.t3, self.T.t4, self.T.t5, self.T.t6]
        lengths = [self.T.l1, self.T.l2, self.T.l3, self.T.l4, self.T.l5, self.T.l6]

        sub_dict = {self.T.ts1: angles[0], self.T.ts2: angles[1], self.T.ts3: angles[2],
            self.T.ts4: angles[3], self.T.ts5: angles[4], self.T.ts6: angles[5], self.T.ls1: lengths[0], self.T.ls2: lengths[1], self.T.ls3: lengths[2],
            self.T.ls4: lengths[3], self.T.ls5: lengths[4], self.T.ls6: lengths[5]}

        for joint in range(1, 7):
            torque = 0
            self.set_symbolic_mass_matrix()
            self.set_numerical_mass_matrix()

            for j in range(6):
                torque += self.num_mass_matrix[joint-1,j] * self.angular_accelerations[j]
            
            for j in range(6):
                for k in range(6):
                    torque += (self.sym_mass_matrix[joint-1, j].diff(sym_angles[k]) - 0.5 * self.sym_mass_matrix[j, k].diff(sym_angles[joint-1])) * self.angular_velocities[j] * self.angular_velocities[k]
            
            for s in range(6):
                H_ib = sp.Matrix([
                    [1, 0, 0, 0],
                    [0, 1, 0, 0],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]
                ])

                for j in range(s+1):
                    H_ib = H_ib @ self.T.s_joint_transforms[j]

                p_scomb = (H_ib @ self.T.s_com_to_joint_transforms[s])[:3, 3]

                torque -= (self.masses[s] * self.gravity @ p_scomb.diff(sym_angles[joint-1]))[0]
            
            torque = torque.evalf(subs=sub_dict)
            print(f"Torque at joint {joint} = {torque} Nm")
            torques[joint-1] = torque.evalf(subs=sub_dict)

        
        self.torques = torques
