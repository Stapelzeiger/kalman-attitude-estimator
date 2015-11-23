import sympy as sp
import sympy.utilities.codegen as cg
import sympy_util


def clean_c_code(c_code, use_single_float=True, static_inline=True):
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
    c_code = c_code.replace('double *__in__', 'const double *__in__')
    c_code = c_code.replace('__in__', '')
    if use_single_float:
        c_code = convert_to_float(c_code)
    if static_inline:
        c_code = c_code.replace('void ', 'static inline void ')
    return c_code


def generate_c_func(name, expr, in_args, column_major_storage=True, **kwargs):
    """ generates C function code for a sympy exrpession
        name is the name of the function
        expr is the matrix expression
        in_args is a list of tuples containing a variable-name string and the
        variable symbol. The symbol can be a matrix or a scalar.
        The sequence of the arguments will be the sequence of in_args and the
        output (called 'out') is last
    """
    if column_major_storage:
        # generated C code matrix access is in row-major order
        # [source: sympy/printing/ccode.py:215, _print_MatrixElement()]
        # this is why we transpose the matrices in column-major mode
        expr = expr.transpose()

    subs_tab = []
    arg_list = []

    for argname, arg_sym in in_args:
        if isinstance(arg_sym, sp.Matrix):
            if column_major_storage:
                arg_sym = arg_sym.transpose()
            new_sym = sp.MatrixSymbol('__in__' + argname, *arg_sym.shape)
            subs_tab += sympy_util.matrix_subs(arg_sym, new_sym)
            arg_list.append(cg.InputArgument(new_sym, dimensions=new_sym.shape))
        else:
            new_sym = sp.symbols(argname)
            subs_tab.append((arg_sym, new_sym))
            arg_list.append(cg.InputArgument(new_sym))

    out_sym = sp.MatrixSymbol('out', expr.shape[0], expr.shape[1])
    out_arg = cg.OutputArgument(out_sym, out_sym, expr.subs(subs_tab), dimensions=out_sym.shape)
    arg_list.append(out_arg)

    no_return_val = []
    no_local_vars = []

    code_gen = cg.get_code_generator("C", "projectname")
    routines = [cg.Routine(name, arg_list, no_return_val, no_local_vars)]
    [(c_name, c_code), (h_name, c_header)] = code_gen.write(routines, "prefix", header=False)
    c_code = clean_c_code(c_code, **kwargs)
    return c_code
