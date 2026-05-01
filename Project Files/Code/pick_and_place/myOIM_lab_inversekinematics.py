from math import pi
import os

import numpy as np
import Sofa
from numpy import linalg as la
from qpsolvers import Problem, solve_problem


def getTorques(W, dq_free, iE, iA, q_s, q_t, q_e, q_a, weights=None):
    """
    Optimization-based Inverse Method (OIM) using Damped Least Squares.
    Modified to increase tracking accuracy and reduce lag.
    """
    # --- Step 1: Extract Projections ---
    Wea = W[iE, :][:, iA]
    Waa = W[iA, :][:, iA]
    de_free = np.asarray(dq_free[iE], dtype=float)
    da_free = dq_free[iA]
    q_t = np.asarray(q_t, dtype=float)
    q_e = np.asarray(q_e, dtype=float)
    weights = (
        np.asarray(weights, dtype=float)
        if weights is not None
        else np.ones(len(iE), dtype=float)
    )
    weighted_Wea = weights[:, None] * Wea
    weighted_error = weights * (de_free + q_e - q_t)

    # --- Step 2: Build OIM Matrices with High-Accuracy Tuning ---
    mu = 1e-5
    P = weighted_Wea.T @ weighted_Wea + mu * np.eye(len(iA))

    gain = 1.0
    q = weighted_Wea.T @ (gain * weighted_error)

    # --- Step 3: Minimal Regularization ---
    weight = 0.001
    P += weight * la.norm(P) / la.norm(Waa) * Waa

    # --- Step 4: Motor Constraints ---
    lb = la.solve(Waa, np.array([-pi, -pi, -pi, -pi]) - q_a - da_free)
    ub = la.solve(Waa, np.array([pi, pi, pi, pi]) - q_a - da_free)

    A, b, G, h = None, None, None, None

    try:
        problem = Problem(P, q, G, h, A, b, lb, ub)
        solution = solve_problem(problem, solver="clarabel")
        if solution.x is not None:
            return solution.x
    except Exception as e:
        Sofa.msg_error(os.path.basename(__file__), "OIM Solver Error: " + str(e))

    return [0.0, 0.0, 0.0, 0.0]
