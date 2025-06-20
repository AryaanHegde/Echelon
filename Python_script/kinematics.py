import numpy as np
import math

class Forward:
    def __init__(self, angles, lengths):
        self.angles = angles
        self.lengths = lengths
        self.deg2rad()

    def deg2rad(self):
        for idx in range(len(self.angles)):
            self.angles[idx] = self.angles[idx] * math.pi / 180

    def compute_homogeneous_transform(self):
        angle_123 = sum(self.angles)
        angle_12 = angle_123 - self.angles[2]
        angle_1 = angle_12 - self.angles[1]
        a_1 = self.lengths[0]
        a_2 = self.lengths[1]
        a_3 = self.lengths[2]
        c_123 = math.cos(angle_123)
        s_123 = math.sin(angle_123)
        c_12 = math.cos(angle_12)
        s_12 = math.sin(angle_12)
        c_1 = math.cos(angle_1)
        s_1 = math.sin(angle_1)

        hom_transform = np.array([
            [c_123, -s_123, 0, a_1 * c_1 + a_2 * c_12 + a_3 * c_123],
            [s_123, c_123, 0, a_1 * s_1 + a_2 * s_12 + a_3 * s_123],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])

        return hom_transform
    
    def get_ee_state(self):
        hom_transform = self.compute_homogeneous_transform()
        pos = hom_transform[:2, 3]
        orientation = hom_transform[:2, :2]
        return [pos, orientation]


class Inverse:
    def __init__(self, lengths, pos, phi=0):
        self.x_ee, self.y_ee = pos
        self.lengths = lengths
        self.phi = phi
        self.deg_to_rad()
        self.a_1, self.a_2, self.a_3 = self.lengths
        self.angles = [0, 0, 0]

    def deg_to_rad(self):
        self.phi = self.phi * math.pi / 180

    def refactor_angles(self):
        prev = 0
        for idx in range(len(self.angles)):
            cur = self.angles[idx]
            self.angles[idx] = prev + cur
            prev += cur

    def get_joint_angles(self):
        for phi_deg in range(-180, 181, 5):
            phi_rad = math.radians(phi_deg)
            x_1 = self.x_ee - self.a_3 * math.cos(phi_rad)
            y_1 = self.y_ee - self.a_3 * math.sin(phi_rad)
            dist = math.sqrt(x_1 ** 2 + y_1 ** 2)

            try:
                val = (dist**2 - self.a_1**2 - self.a_2**2) / (2 * self.a_1 * self.a_2)
                if abs(val) > 1:
                    continue
                theta_2 = math.acos(val)
                beta = math.atan2(self.a_2 * math.sin(theta_2), self.a_1 + self.a_2 * math.cos(theta_2))
                gamma = math.atan2(y_1, x_1)
                theta_1 = gamma - beta
                theta_3 = phi_rad - theta_2 - theta_1
                self.angles = [theta_1, theta_2, theta_3]
                self.phi = phi_rad
                for idx in range(len(self.angles)):
                    self.angles[idx] = math.degrees(self.angles[idx])
                return
            except ValueError:
                continue

        print("Point out of workspace")
        self.angles = [0, 0, 0]


