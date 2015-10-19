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
    # sp.pprint(val)
    res_ = sp.symbols("res_", real=True)
    # sp.printing.print_ccode(res_ = val)
    print(val)

def write_c(val):
    from sympy.utilities.codegen import codegen
    [(c_name, c_code), (h_name, c_header)] = codegen(
        ("f", val), "C", "test", header=False, empty=False)
    print(c_code)

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

q0, q1, q2, q3 = sp.symbols("q0 q1 q2 q3", real=True)
attitude = sp.Matrix([[q0], [q1], [q2], [q3]])

gbx, gby, gbz = sp.symbols("gbx, gby, gbz", real=True)
gyro_bias = sp.Matrix([[gbx], [gby], [gbz]])

x = attitude.col_join(gyro_bias)
display("x", x)

# gyro
gx, gy, gz = sp.symbols("gx gy gz", real=True)
gyro = sp.Matrix([[gx], [gy], [gz]])

rate = gyro - gyro_bias
display("rate", rate)

rate_quat = rate.row_insert(0, sp.Matrix([0]))

attitude_dot = 1/2*quatmult(attitude, rate_quat) # quaternion kinemamtics
gyro_bias_dot = sp.zeros(3, 1) # constant gyro bias model

x_dot = attitude_dot.col_join(gyro_bias_dot)
x_dot = sp.simplify(x_dot)

display("x_dot", x_dot)

display("jacobian", x_dot.jacobian(x))


expected_meas = sp.Matrix([0, 0, 9.81])
h = rotate_by_quaternion(expected_meas, quatconj(attitude))
H = h.jacobian(x)

display("h", h)

display("H", H)

# x_var = cg.InputArgument(x, datatype='float')
# h_out_var = cg.OutputArgument()
# h_fn = cg.Routine("h", )
h_sym = sp.MatrixSymbol('h', 3, 1)
q_sym = sp.MatrixSymbol('q', 4, 1)
print(cg.InputArgument(q0, datatype=cg.DataType("float", "", "float", "")).name)
argseq = [h_sym, q_sym, q0, q1, q2, q3]

[(c_name, c_code), (h_name, c_header)] = cg.codegen(("fn", sp.Eq(h_sym, sp.Matrix([0, 0, q_sym[1]]))), "C", "test", argument_sequence=argseq)
print(c_code)


cfloat_type = cg.DataType("float", "", "float", "")
cconstfloat_type = cg.DataType("const float", "", "float", "")
return_val = []
arg_list = [cg.InputArgument(q_sym, dimensions=(3, 1), datatype=cconstfloat_type),
            cg.OutputArgument(h_sym, h_sym, sp.Matrix([0, 0, q_sym[1]]), dimensions=(3, 1), datatype=cfloat_type)]
local_vars = []
routines = [cg.Routine("fn", arg_list, return_val, local_vars)]
code_gen = cg.get_code_generator("C", "projectname")
[(c_name, c_code), (h_name, c_header)] = code_gen.write(routines, "prefix")
print(c_code)
# write_c(H)