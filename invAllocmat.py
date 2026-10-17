import sympy as sp

d, k = sp.symbols("d k")
A = sp.Matrix(
    [
        [1, 1, 1, 1],
        [-d, -d, d, d],
        [-d, d, d, -d],
        [k, -k, k, -k],
    ]
)
A_inv = sp.simplify(A.inv())
sp.pprint(A_inv, use_unicode=True)
