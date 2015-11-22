
# coding: utf-8

# ## Extended Kalman Filter derivations for a simple attitude estimation model
# 
# estimates:
# - attitude (quaternion)
# 
# based on:
# - gyro
# - accelerometer ("up" sensor)
# 
# model:
# - direct gyro "control" input for state propagation
# - no acceleration -> only gravity which is used to compensate for gyro drift (measurement input)
# 

# In[ ]:

from ekf_common import *
import sympy as sp
#sp.init_printing() # use this for fancy Latex rendering
from IPython.display import Latex, display


# In[ ]:

delta_t = sp.symbols("Delta_t", real=True)

q0, q1, q2, q3 = sp.symbols("q0 q1 q2 q3", real=True)
attitude = sp.Matrix([[q0], [q1], [q2], [q3]])

gbx, gby, gbz = sp.symbols("gbx, gby, gbz", real=True)
gyro_bias = sp.Matrix([[gbx], [gby], [gbz]])

x = attitude.col_join(gyro_bias)
x = attitude
display("x", x)

# gyro
gx, gy, gz = sp.symbols("gx gy gz", real=True)
gyro = sp.Matrix([[gx], [gy], [gz]])
u = gyro

rate = gyro - gyro_bias
rate = gyro
display("rate", rate)

rate_quat = rate.row_insert(0, sp.Matrix([0]))

attitude_dot = 1/2*quatmult(attitude, rate_quat) # quaternion kinemamtics
gyro_bias_dot = sp.zeros(3, 1) # constant gyro bias model
gyro_bias_dot = - gyro_bias * 0.1

x_dot = attitude_dot.col_join(gyro_bias_dot)
x_dot = attitude_dot
x_dot = sp.simplify(x_dot)

display("x_dot", x_dot)

f = x_dot * delta_t + x
F = f.jacobian(x)

display("f", f)
display("F", F)


expected_meas = sp.Matrix([0, 0, 9.81])
h = rotate_by_quaternion(expected_meas, quatconj(attitude))
H = h.jacobian(x)

display("h", h)
display("H", H)


# In[ ]:

import imp, ekf_common
imp.reload(ekf_common)
c_code = ekf_common.ekf_generate_c_code(f, F, h, H, x, u, [delta_t])
print(c_code)


# In[ ]:

c_file = open('ekf_gyro_acc.h', 'w+')
c_file.write(c_code)
c_file.close()


# In[ ]:



