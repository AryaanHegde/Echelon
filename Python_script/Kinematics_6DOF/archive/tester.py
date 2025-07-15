import math
import numpy as np
lengths = [3, 2, 5, 5, 2, 1]
l1, l2, l3, l4, l5, l6 = lengths

x_eb = (l2 + l3 + l4 + l5 + l6) * math.cos(np.pi/4)
y_eb = 0
z_eb = l1 + (l2 + l3 + l4 + l5 + l6) * math.sin(np.pi/4)
roll = 0



print(x_eb)
print(y_eb)
print(z_eb)