"""
Extended Kalman Filter derivations for a simple attitude estimation model

 estimates:
 - attitude (reference + estimated error quaternion)
 - gyro bias

 based on:
 - gyro
 - accelerometer ("up" sensor)
 - magnetometer ("north" sensor)

 model:
 - direct gyro "control" input for state propagation
 - no acceleration -> only gravity which is used to compensate for gyro drift (measurement input)
 - magnetometer projected on horizontal plane points north (direction x+)
"""

import logging
import quaternion
import c_code_gen
import sympy as sp
import argparse
from sympy_util import matrix_subs

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('-l', '--loglevel', default='WARNING')
    args = parser.parse_args()
    logging.basicConfig(level=args.loglevel.upper())


def display(name, val):
    logging.debug("{} = {}".format(name, val))


delta_t = sp.symbols("Delta_t", real=True)


# Gyro Model
gx, gy, gz = sp.symbols("g_x g_y g_z", real=True)
gyro = sp.Matrix([gx, gy, gz])

gnx, gny, gnz = sp.symbols("gn_x gn_y gn_z", real=True)
gyro_noise = sp.Matrix([gnx, gny, gnz])

gbx, gby, gbz = sp.symbols("gb_x, gb_y, gb_z", real=True)
gyro_bias = sp.Matrix([gbx, gby, gbz])

gb_hatx, gb_haty, gb_hatz = sp.symbols("gbhat_x, gbhat_y, gbhat_z", real=True)
gyro_bias_hat = sp.Matrix([gb_hatx, gb_haty, gb_hatz])

omega = gyro - gyro_bias - gyro_noise
omega_quat = sp.Matrix([0]).col_join(omega)
omega_hat = gyro - gyro_bias_hat
omega_hat_quat = sp.Matrix([0]).col_join(omega_hat)

# Attitude representation
# q = q_ref * delta_q(a)
# attitude reference quaternion q_ref
q0, q1, q2, q3 = sp.symbols("q0 q1 q2 q3", real=True)
q_ref = sp.Matrix([[q0], [q1], [q2], [q3]])
# attitude error quaternion delta_q
# The attitude quaternion delta_q is parametrized by a three-dimensional
# attitude representation. The representation chosen is a Gibbs-vector scaled by
# a factor of two. A Gibbs-vector is defined as the normalized rotation axis
# scaled by the tangent of half the rotation angle ( g = n*tan(theta/2) )
# therefore g = [q1; q2; q3] / q0
# and a = 2 * [q1; q2; q3] / q0
# or q = 1/sqrt(4 + norm(a)^2) * [2; ax; ay; az]
ax, ay, az = sp.symbols("a_x, a_y, a_z")
a = sp.Matrix([ax, ay, az])
delta_q = 1/sp.sqrt(4 + (a.transpose()*a)[0]) * sp.Matrix([2]).col_join(a)
delta_q_r = delta_q[0]
delta_q_v = sp.Matrix(delta_q[1:])

delta_q_dot = 1/2 * quaternion.mult(delta_q, omega_quat) -1/2 * quaternion.mult(omega_hat_quat, delta_q);
delta_q_dot_r = delta_q_dot[0]
delta_q_dot_v = sp.Matrix(delta_q_dot[1:])
a_dot = 2*(delta_q_dot_v/delta_q_r - delta_q_v*delta_q_dot_r/delta_q_r**2);
gyro_bias_dot = sp.zeros(3, 1) # constant gyro bias model


x = a.col_join(gyro_bias)
#display("x", x)

x_dot = a_dot.col_join(gyro_bias_dot)
x_dot = sp.simplify(x_dot)
#display("x_dot", x_dot)



f = x + x_dot * delta_t
F = x_dot.jacobian(x)
# taking the expectation
substab = matrix_subs(gyro_bias, gyro_bias_hat)
substab += matrix_subs(gyro_noise, sp.zeros(3))
substab += matrix_subs(a, sp.zeros(3))
F = sp.simplify(F.subs(substab))
phi = sp.eye(len(x)) + F*delta_t


display("f", f)
display("F", F)
display("phi", phi)



def meas_ref_q_from_v(v):
    v_n = v.normalize()

qmr0, qmr1, qmr2, qmr3 = sp.symbols('qmr0 qmr1 qmr2 qmr3')
q_meas_ref = sp.Matrix([qmr0, qmr1, qmr2, qmr3])

def measurement_from_vect(v_body):
    #v_body = v_body.normalize()
    v_rot = quaternion.rotate_vect(v_body, quaternion.mult(quaternion.conj(q_meas_ref), q_ref))
    return sp.Matrix([v_rot[1], v_rot[2]])

vg_body = sp.simplify(quaternion.rotate_vect(sp.Matrix([0, 0, 1]), quaternion.conj(quaternion.mult(q_ref, delta_q))))
tmp =  measurement_from_vect(vg_body).subs(matrix_subs(a, sp.Matrix([0, 0, 0])) + matrix_subs(q_meas_ref, sp.Matrix([sp.cos(sp.pi/4), 0, sp.sin(sp.pi/4), 0])))
display("vg_body", sp.simplify(tmp))


#expected_meas = sp.Matrix([0, 0, 9.81])
#h = quaternion.rotate_vect(expected_meas, quaternion.conj(attitude))
#H = h.jacobian(x)

#display("h", h)
#display("H", H)



# reference dynamics
#attitude_dot = 1/2*quaternion.mult(attitude, rate_quat) # quaternion kinemamtics



# import imp, ekf_common
# imp.reload(ekf_common)
# c_code = ekf_common.ekf_generate_c_code(f, F, h, H, x, u, [delta_t])
# #print(c_code)


# north = sp.Matrix([1, 0, 0])
# r0, r1, r2, r3 = sp.symbols("r0 r1 r2 r3", real=True)
# ref = sp.Matrix([[r0], [r1], [r2], [r3]])
# north_vect_in_ref = quaternion.rotate_vect(north, quaternion.mult(ref, quaternion.conj(attitude)))
# h_mag = sp.Matrix([sp.atan2(north_vect_in_ref[1], north_vect_in_ref[0])])
# H_mag = h_mag.jacobian(x)
# ref_subs = list(zip([r0, r1, r2, r3], [q0, q1, q2, q3])) + [(q0**2 + q1**2 + q2**2 + q3**2, 1)]
# display(ref_subs)
# #display("h_mag", h_mag)
# #display("H_mag", H_mag)
# h_mag = h_mag.subs(ref_subs)
# H_mag = H_mag.subs(ref_subs)
# display("h_mag", h_mag)
# display("H_mag", H_mag)

# z0, z1, z2 = sp.symbols("z0 z1 z2", real=True)
# z = sp.Matrix([[z0], [z1], [z2]])
# z_inertial = quaternion.rotate_vect(z, attitude)
# measurement_transform = sp.Matrix([sp.atan2(z_inertial[1], z_inertial[0])])
# display(measurement_transform)
# measurement_transform_c_code = ekf_common.generate_c_code('meas_transf', measurement_transform)
# z_inertial_c_code = ekf_common.generate_c_code('z_inertial', z_inertial)


# c_code = ekf_generate_c_code(f, F, h_mag, H_mag, x, u, [delta_t]) + measurement_transform_c_code + z_inertial_c_code
# print(c_code)


# c_file = open('ekf_gyro_mag.h', 'w+')
# c_file.write(c_code)
# c_file.close()

