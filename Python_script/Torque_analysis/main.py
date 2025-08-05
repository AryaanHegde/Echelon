import sympy as sp

g = sp.transpose(sp.Matrix([0, 0, -9.81]))
p = sp.Matrix([1, 2, 3])
print((g @ p)[0])