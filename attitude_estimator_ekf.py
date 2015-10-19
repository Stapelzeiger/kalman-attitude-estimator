import sympy as sp
import sympy.utilities.codegen as cg
"""
Extended Kalman Filter derivations for a simple attitude estimation model

estimates:
- attitude
- gyro bias

based on:
- gyro
- accelerometer ("up" sensor)
- magnetometer

model:
- direct gyro "control" input for state propagation
- no acceleration -> only gravity which is used to compensate for gyro drift (measurement input)

"""


def display(var, val):
    print()
    print(var, "=")
    print(val)

def generate_c_code(fn, expr, use_single_float=True):

    def convert_to_float(c_code):
        c_code = c_code.replace('double', 'float')
        c_code = c_code.replace('pow', 'powf')
        return c_code

    from sympy.utilities.codegen import codegen
    [(c_name, c_code), (h_name, c_header)] = codegen(
        (fn, expr), "C", "", header=False, empty=False)

    c_code = filter(lambda line: not line.startswith('#include'), c_code.split('\n'))
    c_code = '\n'.join(c_code)
    c_code = c_code.replace('double *in', 'const double *in')
    if use_single_float:
        c_code = convert_to_float(c_code)
    c_code = 'static ' + c_code
    return c_code


def rotate_by_quaternion(v, q):
    qv = sp.Matrix([0]).col_join(v)
    qv_rot = quatmult(quatmult(q, qv), quatconj(q))
    qv_rot.row_del(0)
    return qv_rot


def quatconj(q):
    qc = sp.Matrix([q[0], -q[1], -q[2], -q[3]])
    return qc


def quatmult(q1, q2):
    q = q1.copy()
    q[0] = q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3]
    q[1] = q1[0]*q2[1] + q1[1]*q2[0] + q1[2]*q2[3] - q1[3]*q2[2]
    q[2] = q1[0]*q2[2] - q1[1]*q2[3] + q1[2]*q2[0] + q1[3]*q2[1]
    q[3] = q1[0]*q2[3] + q1[1]*q2[2] - q1[2]*q2[1] + q1[3]*q2[0]
    return q


delta_t = sp.symbols("delta_t", real=True)

q0, q1, q2, q3 = sp.symbols("q0 q1 q2 q3", real=True)
attitude = sp.Matrix([[q0], [q1], [q2], [q3]])

gbx, gby, gbz = sp.symbols("gbx, gby, gbz", real=True)
gyro_bias = sp.Matrix([[gbx], [gby], [gbz]])

x = attitude.col_join(gyro_bias)
display("x", x)

# gyro
gx, gy, gz = sp.symbols("gx gy gz", real=True)
gyro = sp.Matrix([[gx], [gy], [gz]])
u = gyro

rate = gyro - gyro_bias
display("rate", rate)

rate_quat = rate.row_insert(0, sp.Matrix([0]))

attitude_dot = 1/2*quatmult(attitude, rate_quat) # quaternion kinemamtics
gyro_bias_dot = sp.zeros(3, 1) # constant gyro bias model

x_dot = attitude_dot.col_join(gyro_bias_dot)
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


# C matrix access is in row-major order
# [source: sympy/printing/ccode.py:215, _print_MatrixElement()]

out_sym = sp.MatrixSymbol('h_out', len(h), 1)
x_sym = sp.MatrixSymbol('in1_x', len(x), 1)
x_subs_tab = [(elem_sym, x_sym[i, 0]) for i, elem_sym in enumerate(x)]
u_sym = sp.MatrixSymbol('in2_u', len(u), 1)
u_subs_tab = [(elem_sym, u_sym[i, 0]) for i, elem_sym in enumerate(u)]
deltat_sym = sp.symbols('in0_delta_t')
subs_tab = x_subs_tab + u_subs_tab + [(delta_t, deltat_sym)]


cfloat_type = cg.DataType("float", "", "float", "")
cconstfloat_type = cg.DataType("const float", "", "float", "")
no_return_val = []
no_local_vars = []
h_arg_list = [cg.InputArgument(x_sym, dimensions=(3, 1), datatype=cconstfloat_type),
              cg.OutputArgument(out_sym, out_sym, h.subs(x_subs_tab), dimensions=(3, 1), datatype=cfloat_type)]
routines = [cg.Routine("h", h_arg_list, no_return_val, no_local_vars)]
code_gen = cg.get_code_generator("C", "projectname")
[(c_name, c_code), (h_name, c_header)] = code_gen.write(routines, "prefix", header=False)
# print(c_code)


filename = 'simple_ekf'
# writing C code the easy way
c_file = open(filename + '.h', 'w+')
c_file.write('// This file has been generated from {}\n'.format(__file__))
c_file.write('// DO NOT EDIT!\n\n')
c_file.write(generate_c_code('H', H.subs(subs_tab)))
c_file.write(generate_c_code('h', h.subs(subs_tab)))
c_file.write(generate_c_code('f', f.subs(subs_tab)))
c_file.write(generate_c_code('F', F.subs(subs_tab)))
