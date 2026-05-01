from math import pi
import os

import numpy as np
import Sofa
from numpy import linalg as la
from qpsolvers import Problem, solve_problem, available_solvers


def getTorques(W, dq_free, iE, iA, q_s, q_t, q_e, q_a, weights=None):
    # --- Step 1: Build the Inverse System Matrices ---
    # We extract the compliance sub-matrix that links Actuators (iA) to the
    # Effector (iE).
    Wea = W[iE, :][:, iA]
    de_free = np.asarray(dq_free[iE], dtype=float)
    q_t = np.asarray(q_t, dtype=float)
    q_e = np.asarray(q_e, dtype=float)
    weights = (
        np.asarray(weights, dtype=float)
        if weights is not None
        else np.ones(len(iE), dtype=float)
    )
    weighted_Wea = weights[:, None] * Wea
    weighted_error = weights * (de_free + q_e - q_t)

    # The objective is to minimize 1/2 * x.T * P * x + q.T * x
    # where x is the vector of motor torques (lambda_a).
    # Add a small diagonal damping term so the QP stays better conditioned on
    # softer scenes such as the deformable gripper transport task.
    mu = 2.5e-5
    P = weighted_Wea.T @ weighted_Wea + mu * np.eye(len(iA))
    q = weighted_Wea.T @ weighted_error

    # --- Step 2: Actuator coupling and free motion ---
    Waa = W[iA, :][:, iA]
    da_free = dq_free[iA]

    A = None
    b = None

    # --- Step 3: Add Energy Term (Regularization) ---
    weight = 0.01
    P += weight * la.norm(P) / la.norm(Waa) * Waa

    # --- Step 4: Motor Displacement Constraints ---
    G = None
    h = None

    lb = la.solve(Waa, np.array([-pi, -pi, -pi, -pi]) - q_a - da_free)
    ub = la.solve(Waa, np.array([pi, pi, pi, pi]) - q_a - da_free)

    torques = [0.0, 0.0, 0.0, 0.0]
    if P is not None:
        try:
            problem = Problem(P, q, G, h, A, b, lb, ub)
            solution = solve_problem(problem, solver="clarabel")
            if solution.x is not None:
                torques = solution.x
        except Exception as e:
            Sofa.msg_error(os.path.basename(__file__), str(e))

    return torques
