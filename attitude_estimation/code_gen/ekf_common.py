import sympy as sp
import sympy.utilities.codegen as cg
import re

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


def clean_c_code(c_code, use_single_float=True):
    def convert_to_float(c_code):
        c_code = c_code.replace('double', 'float')
        c_code = c_code.replace('pow(', 'powf(')
        c_code = c_code.replace('sin(', 'sinf(')
        c_code = c_code.replace('cos(', 'cosf(')
        c_code = c_code.replace('tan(', 'tanf(')
        c_code = c_code.replace('asin(', 'asinf(')
        c_code = c_code.replace('acos(', 'acosf(')
        c_code = c_code.replace('atan2(', 'atan2f(')
        return c_code

    c_code = filter(lambda line: not line.startswith('#include'), c_code.split('\n'))
    c_code = '\n'.join(c_code)
    c_code = c_code.replace('double *in', 'const double *in')
    if use_single_float:
        c_code = convert_to_float(c_code)
    c_code = c_code.replace('void ', 'static inline void ')
    return c_code


def generate_c_code(fn, expr, use_single_float=True, args=None):
    from sympy.utilities.codegen import codegen
    [(c_name, c_code), (h_name, c_header)] = codegen(
        (fn, expr), "C", "", header=False, empty=False, argument_sequence=args)

    c_code = re.sub(r'out_[0-9]{19}', 'out', c_code)
    c_code = clean_c_code(c_code, use_single_float)
    return c_code


def ekf_generate_c_code(f, F, h, H, x, u, f_additional_in=[], h_additional_in=[]):

    # generated C code matrix access is in row-major order
    # [source: sympy/printing/ccode.py:215, _print_MatrixElement()]
    # Since Eigen uses column-major by default we transpose the matrices before
    # generating the C code
    H = H.transpose()
    F = F.transpose()

    f_additional_in_sym = [sp.symbols('in{}_{}'.format(i+2, var.name)) for i, var in enumerate(f_additional_in)]
    f_add_subs_tab = list(zip(f_additional_in, f_additional_in_sym))
    h_additional_in_sym = [sp.symbols('in{}_{}'.format(i+1, var.name)) for i, var in enumerate(h_additional_in)]
    h_add_subs_tab = list(zip(h_additional_in, h_additional_in_sym))
    x_sym = sp.MatrixSymbol('in0_x', len(x), 1)
    x_subs_tab = [(elem_sym, x_sym[i, 0]) for i, elem_sym in enumerate(x)]
    u_sym = sp.MatrixSymbol('in1_u', len(u), 1)
    u_subs_tab = [(elem_sym, u_sym[i, 0]) for i, elem_sym in enumerate(u)]

    subs_tab = x_subs_tab + u_subs_tab + f_add_subs_tab + h_add_subs_tab

    # c_code  = generate_c_code('f', f.subs(subs_tab)) + '\n'
    # c_code += generate_c_code('F', F.subs(subs_tab)) + '\n'
    # c_code += generate_c_code('h', h.subs(subs_tab)) + '\n'
    # c_code += generate_c_code('H', H.subs(subs_tab)) + '\n'

    no_return_val = []
    no_local_vars = []

    f_additional_in_arg = [cg.InputArgument(sym) for sym in f_additional_in_sym]
    h_additional_in_arg = [cg.InputArgument(sym) for sym in h_additional_in_sym]

    f_out_sym = sp.MatrixSymbol('f_out', len(x), 1)
    f_arg_list = [cg.InputArgument(x_sym, dimensions=x_sym.shape),
                  cg.InputArgument(u_sym, dimensions=u_sym.shape)]
    f_arg_list += f_additional_in_arg
    f_arg_list += [cg.OutputArgument(f_out_sym, f_out_sym, f.subs(subs_tab), dimensions=f_out_sym.shape)]

    F_out_sym = sp.MatrixSymbol('F_out', F.shape[0], F.shape[1])
    F_arg_list = [cg.InputArgument(x_sym, dimensions=x_sym.shape),
                  cg.InputArgument(u_sym, dimensions=u_sym.shape)]
    F_arg_list += f_additional_in_arg
    F_arg_list += [cg.OutputArgument(F_out_sym, F_out_sym, F.subs(subs_tab), dimensions=F_out_sym.shape)]

    h_out_sym = sp.MatrixSymbol('h_out', len(h), 1)
    h_arg_list = [cg.InputArgument(x_sym, dimensions=x_sym.shape)]
    h_arg_list += h_additional_in_arg
    h_arg_list += [cg.OutputArgument(h_out_sym, h_out_sym, h.subs(subs_tab), dimensions=h_out_sym.shape)]

    H_out_sym = sp.MatrixSymbol('H_out', H.shape[0], H.shape[1])
    H_arg_list = [cg.InputArgument(x_sym, dimensions=x_sym.shape)]
    H_arg_list += h_additional_in_arg
    H_arg_list += [cg.OutputArgument(H_out_sym, H_out_sym, H.subs(subs_tab), dimensions=H_out_sym.shape)]

    routines = [cg.Routine("f", f_arg_list, no_return_val, no_local_vars),
                cg.Routine("F", F_arg_list, no_return_val, no_local_vars),
                cg.Routine("h", h_arg_list, no_return_val, no_local_vars),
                cg.Routine("H", H_arg_list, no_return_val, no_local_vars)]
    code_gen = cg.get_code_generator("C", "projectname")
    [(c_name, c_code), (h_name, c_header)] = code_gen.write(routines, "prefix", header=False)

    c_code = clean_c_code(c_code, use_single_float=True)

    c_code_head = '// This file has been automatically generated\n'
    c_code_head += '// DO NOT EDIT!\n\n'
    c_code_head += 'const int STATE_DIM = {};\n'.format(len(x))
    c_code_head += 'const int CONTROL_DIM = {};\n'.format(len(u))
    c_code_head += 'const int MEASURE_DIM = {};\n'.format(len(h))
    c_code_head += '\n\n'

    return c_code_head + c_code
