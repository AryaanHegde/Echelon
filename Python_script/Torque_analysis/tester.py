import sympy as sp
from transformations import TransformationsMixin
from jacobian import JacobianMixin
from mass_matrix import MassMatrixMixin
from torque import TorqueMixin
from angle_conversion import*

class Test(TransformationsMixin, JacobianMixin, MassMatrixMixin, TorqueMixin):
    def __init__(self, angles, lengths, angular_velocities, masses, angular_accelerations):
        self.T = TransformationsMixin(angles, lengths)
        self.masses = masses
        self.angular_velocities = rpm_to_rad_s(angular_velocities)
        self.angular_accelerations = rpm_to_rad_s(angular_accelerations)
        self.gravity = sp.transpose(sp.Matrix([0, 0, -9.81]))
        self.num_mass_matrix = None
        self.sym_mass_matrix = None
        self.torques = None
    
    def test_torques(self):
        self.set_torques()
        return self.torques


angles = [0, 0, 0, 0, 0, 0]
lengths = [0.1, 0.25, 0.2, 0.1, 0.1, 0.05]
masses = [0.35, 0.5, 0.5, 0.2, 0.15, 0.15]
angular_velocities = [60, 60, 60, 60, 60, 60]
angular_accelerations = [240, 240, 240, 240, 240, 240]

tester = Test(angles, lengths, angular_velocities, masses, angular_accelerations)
torques = tester.test_torques()
print(torques)