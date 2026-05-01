from qpsolvers import Problem, solve_problem
import numpy as np
from numpy import linalg as la
from math import pi
import os
import Sofa

def getTorques(W, dq_free, iE, iA, q_s, q_t, q_e, q_a):
    """
    Optimization-based Inverse Method (OIM) using Damped Least Squares.
    Modified to increase tracking accuracy and reduce lag.
    """
    # --- Step 1: Extract Projections ---
    Wea = W[iE, :][:, iA]
    Waa = W[iA, :][:, iA]
    de_free = dq_free[iE]
    da_free = dq_free[iA]
    
    # --- Step 2: Build OIM Matrices with High-Accuracy Tuning ---
    # mu: Damping factor. We reduce this to 1e-5 to minimize the 'brake' effect.
    mu = 1e-5 
    P = Wea.T @ Wea + mu * np.eye(4)
    
    # gain: Increases the 'pull' toward the target. 
    # Use 1.0 for standard tracking, or increase (e.g., 1.5) to close the gap faster.
    gain = 0.85
    
    # Gradient (q): Calculates the direction to move.
    q = Wea.T @ (gain * (de_free + q_e - q_t))

    # --- Step 3: Minimal Regularization ---
    # We reduce the energy weight significantly (from 0.01 to 0.0001).
    # This ensures accuracy is prioritized over minimizing motor effort.
    weight = 0.001 
    P += weight * la.norm(P) / la.norm(Waa) * Waa

    # --- Step 4: Motor Constraints ---
    # Ensures the motors stay within physical bounds [-pi, pi]
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
    
    return [0., 0., 0., 0.]