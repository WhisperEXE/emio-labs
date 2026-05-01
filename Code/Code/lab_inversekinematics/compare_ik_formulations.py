import os
import sys
import argparse
import csv
import time
import numpy as np
from math import pi, sin, cos

sys.path.append(os.path.dirname(os.path.realpath(__file__)) + "/../../")

from utils import getListFromArgs
from utils.header import addHeader, addSolvers
from parts.emio import Emio, getParserArgs
from parts.controllers.assemblycontroller import AssemblyController

import Sofa
import Sofa.Core
import Sofa.SoftRobotsInverse
import myQP_lab_inversekinematics as myQP
import myOIM_lab_inversekinematics as myOIM
import parameters

# --- Safe Dynamic Model Imports ---
try:
    import modeling_techniques.beam_models.standard_beam as beam
except ImportError:
    beam = None

# ---------------------------------------------------------
# Formulation II & III Solver Wrapper
# ---------------------------------------------------------
class MyCustomInverseSolver(Sofa.SoftRobotsInverse.QPInverseProblemSolver):
    def __init__(self, emio, sensor, target, effector, getTorques, *args, **kwargs):
        Sofa.SoftRobotsInverse.QPInverseProblemSolver.__init__(self, *args, **kwargs)
        self.name = "ConstraintSolver"
        self.emio = emio
        self.sensor = sensor
        self.target = target.getMechanicalState()
        self.effector = effector.getMechanicalState()
        self.assembly = emio.addObject(AssemblyController(emio))
        self.getTorques = getTorques
        self.lastTorques = None

    def solveSystem(self):
        W = self.W()
        dfree = self.dfree()
        torques = self.lambda_force()
        iE, iA = [4, 5, 6], [0, 1, 2, 3]
        q_a = [self.emio.motors[i].JointActuator.angle.value for i in range(4)]
        dq_free = np.copy(dfree)
        dq_free[iA] -= q_a

        if self.assembly.done:
            try:
                raw_torques = self.getTorques(W=W, dq_free=dq_free, iE=iE, iA=iA,
                                               q_s=self.sensor.position.value[0][0:3],
                                               q_t=self.target.position.value[0][0:3],
                                               q_e=self.effector.position.value[0][0:3],
                                               q_a=q_a)
                # --- SAFETY GOVERNOR ---
                # Prevents un-damped QP from generating infinite torque and exploding the mesh
                torques[iA] = np.clip(raw_torques, -1000, 1000)
            except Exception:
                if self.lastTorques is not None:
                    torques[iA] = np.copy(self.lastTorques)
        self.lastTorques = np.copy(torques[iA])
        return True

# ---------------------------------------------------------
# Automated Target Mover (Creates a Figure-8 trajectory)
# ---------------------------------------------------------
class AutomatedTargetMover(Sofa.Core.Controller):
    def __init__(self, target_mo):
        Sofa.Core.Controller.__init__(self)
        self.target_mo = target_mo
        self.step = 0
        self.base_y = target_mo.position.value[0][1]

    def onAnimateBeginEvent(self, e):
        self.step += 1
        t = self.step * 0.01 
        
        x = 15 * sin(t)
        y = self.base_y + 10 * sin(t * 2)
        
        pos = np.copy(self.target_mo.position.value)
        pos[0][0] = x
        pos[0][1] = y
        self.target_mo.position.value = pos

# ---------------------------------------------------------
# Benchmarking Logger
# ---------------------------------------------------------
class BenchmarkLogger(Sofa.Core.Controller):
    def __init__(self, emio, target_mo, filename):
        Sofa.Core.Controller.__init__(self)
        self.emio = emio
        self.target_mo = target_mo
        self.effector_mo = emio.effector.getMechanicalState()
        
        self.file = open(filename, 'w', newline='')
        self.writer = csv.writer(self.file)
        self.writer.writerow(['Step', 'ComputeTime_ms', 'Target_X', 'Effector_X', 'Error_Norm', 'Motor0_rad'])
        
        self.step = 0
        self.last_time = time.time()
        print(f"BENCHMARK START: Logging to {filename}")

    def onAnimateEndEvent(self, e):
        self.step += 1
        curr_time = time.time()
        dt_ms = (curr_time - self.last_time) * 1000
        self.last_time = curr_time

        t_pos = self.target_mo.position.value[0][0:3]
        e_pos = self.effector_mo.position.value[0][0:3]
        
        # Calculate 3D Euclidean Error
        error = np.linalg.norm(np.array(t_pos) - np.array(e_pos))
        m0 = self.emio.motors[0].JointActuator.angle.value

        self.writer.writerow([self.step, round(dt_ms, 2), round(t_pos[0], 2), round(e_pos[0], 2), round(error, 3), round(m0, 3)])
        self.file.flush()

        if self.step >= 400:
            print(f"Benchmark reached 400 steps. Automatically pausing.")
            self.target_mo.getContext().getRootContext().animate.value = False

# ---------------------------------------------------------
# Scene Creation
# ---------------------------------------------------------
def createScene(rootnode):
    parser = argparse.ArgumentParser()
    parser.add_argument('--formulation', type=int, choices=[1, 2, 3], default=1, 
                        help="1: Integrated (SOFA), 2: Reduced QP, 3: OIM-Inspired")
    args, _ = parser.parse_known_args()

    settings, modelling, simulation = addHeader(rootnode, inverse=True)
    rootnode.dt, rootnode.gravity = 0.03, [0., -9810., 0.]
    addSolvers(simulation)

    emio = Emio(name="Emio", legsName=["blueleg"], legsModel=["beam"], 
                legsPositionOnMotor=["counterclockwisedown", "clockwisedown", "counterclockwisedown", "clockwisedown"],
                legsYoungModulus=[parameters.youngModulus], centerPartName="bluepart", centerPartType="rigid", extended=True)
    
    for leg in emio.legs:
        beam.create_model(leg, argparse.Namespace(legName="blueleg"), parameters, [0,0,0])

    simulation.addChild(emio)
    emio.attachCenterPartToLegs()

    effector = emio.effector
    effector.addObject("MechanicalObject", template="Rigid3", position=[0, 0, 0, 0, 0, 0, 1])
    effector.addObject("RigidMapping", index=0)

    effectorTarget = modelling.addChild('Target')
    effectorTarget.addObject('EulerImplicitSolver', firstOrder=True)
    effectorTarget.addObject('CGLinearSolver', iterations=50, tolerance=1e-10, threshold=1e-10)
    effectorTarget.addObject('MechanicalObject', template='Rigid3', position=[0, -170, 0, 0, 0, 0, 1], showObject=True)

    sensor = emio.effector.getMechanicalState()

    # --- APPLY THE CORRECT FORMULATION BASED ON THE BRIEFING ---
    if args.formulation == 1:
        print("Using Formulation I: Integrated Inverse Formulation (Native SOFA)")
        emio.addInverseComponentAndGUI(targetMechaLink=effectorTarget.getMechanicalState().position.linkpath, orientationWeight=0, withGUI=False)
        filename = "benchmark_form1.csv"
        
    elif args.formulation == 2:
        print("Using Formulation II: Reduced Actuator-Space Formulation (QP)")
        rootnode.removeObject(rootnode.ConstraintSolver)
        rootnode.addObject(MyCustomInverseSolver(emio, sensor, effectorTarget, effector, myQP.getTorques))
        # FIXED: Targets itself to mimic qp_inversekinematics.py perfectly
        emio.addInverseComponentAndGUI(targetMechaLink=effector.getMechanicalState().position.linkpath, withGUI=False)
        filename = "benchmark_form2.csv"
        
    elif args.formulation == 3:
        print("Using Formulation III: OIM-Inspired Formulation")
        rootnode.removeObject(rootnode.ConstraintSolver)
        rootnode.addObject(MyCustomInverseSolver(emio, sensor, effectorTarget, effector, myOIM.getTorques))
        # FIXED: Uses orientationWeight=0 to mimic oim_inversekinematics.py perfectly
        emio.addInverseComponentAndGUI(targetMechaLink=effectorTarget.getMechanicalState().position.linkpath, orientationWeight=0, withGUI=False)
        filename = "benchmark_form3.csv"

    # Add Test Controllers
    rootnode.addObject(AutomatedTargetMover(effectorTarget.getMechanicalState()))
    rootnode.addObject(BenchmarkLogger(emio, effectorTarget.getMechanicalState(), filename))
    
    return rootnode