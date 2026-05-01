from qpsolvers import Problem, solve_problem, available_solvers
import os
import Sofa
import numpy as np
from numpy import linalg as la
from math import pi

def getTorques(W, dq_free, iE, iA, q_s, q_t, q_e, q_a):
    # --- Step 1: Build the Inverse System Matrices ---
    # We extract the compliance sub-matrix that links Actuators (iA) to the Effector (iE)
    Wea = W[iE, :][:, iA]
    de_free = dq_free[iE]
    
    # The objective is to minimize 1/2 * x.T * P * x + q.T * x
    # where x is the vector of motor torques (lambda_a)
    P = Wea.T @ Wea
    q = Wea.T @ (de_free + q_e - q_t)

    # --- Step 2: Constraint to block Motor 0 (angle = -0.5 rad) ---
    # We use A*x = b to enforce a specific position for an actuator
    # For motor 0 (index 0 in iA), the constraint is: 
    # delta_a[0] + H_a[0]*dq_free + Waa[0]*lambda_a = target_angle
    Waa = W[iA, :][:, iA]
    da_free = dq_free[iA]
    
    # A corresponds to the row of Waa for the motor we want to block
    A = np.zeros((1, 4))
    A[0, :] = Waa[0, :]
    
    # b is the difference between target angle (-0.5) and current free motion
    b = np.array([-0.5 - (q_a[0] + da_free[0])])

    # --- Step 3: Add Energy Term (Regularization) ---
    # This prevents the robot from taking "extreme" paths if multiple solutions exist
    weight = 0.01 
    P += weight * la.norm(P) / la.norm(Waa) * Waa

    # --- Step 4: Motor Displacement Constraints ---
    # We add bounds to ensure the motors stay within physical limits (-pi to pi)
    # lb <= lambda_a <= ub
    # We must solve for lambda_a: u_min <= q_a + da_free + Waa*lambda_a <= u_max
    
    # G and h can be used for general inequalities (G*x <= h), 
    # but for simple box limits, lb and ub are more efficient.
    G = None
    h = None
    
    # Solving the inequality for lambda_a:
    lb = la.solve(Waa, np.array([-pi, -pi, -pi, -pi]) - q_a - da_free)
    ub = la.solve(Waa, np.array([pi, pi, pi, pi]) - q_a - da_free)

    torques = [0., 0., 0., 0.]
    if P is not None:
        try:
            problem = Problem(P, q, G, h, A, b, lb, ub)
            # Clarabel is great for handling these soft robot constraints
            solution = solve_problem(problem, solver="clarabel")
            if solution.x is not None:
                torques = solution.x
        except Exception as e:
            Sofa.msg_error(os.path.basename(__file__), str(e))
            # Fallback to no torque if solver fails to converge
    
    return torques