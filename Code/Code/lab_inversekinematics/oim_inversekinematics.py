import os
import sys
import numpy as np
from math import pi

# --- Path Setup ---
sys.path.append(os.path.dirname(os.path.realpath(__file__)) + "/../../")

from utils import getListFromArgs
from utils.header import addHeader, addSolvers
from parts.emio import Emio, getParserArgs
from parts.controllers.assemblycontroller import AssemblyController
from parts.controllers.trackercontroller import DotTracker

import Sofa
import Sofa.Core
import Sofa.SoftRobotsInverse
import myOIM_lab_inversekinematics as myOIM
import Sofa.ImGui as MyGui
from splib3.numerics import Vec3, vsub
import parameters

# --- Safe Dynamic Model Imports ---
beam = cosserat = tetra = tetra_linear = hyper = nonuniform = None 
try:
    import modeling_techniques.beam_models.standard_beam as beam
    import modeling_techniques.beam_models.cosserat_rod as cosserat
    import modeling_techniques.volume_models.tetra_fem as tetra
    import modeling_techniques.volume_models.tetra_linear_fem as tetra_linear
    import modeling_techniques.volume_models.hyperelastic_fem as hyper
    import modeling_techniques.volume_models.tetra_nonuniform as nonuniform
except ImportError as e:
    print(f"CRITICAL: OIM Model Import Error: {e}")

# Reuse the Direct GUI from your QP logic for Exercise 1
class LabGUIExerciseDirect(Sofa.Core.Controller):
    def __init__(self, root, emio):
        Sofa.Core.Controller.__init__(self)
        self.name = "LabGUIExerciseDirect"
        self.root = root
        self.emio = emio
        MyGui.MoveWindow.setActuators([m.JointActuator.value for m in emio.motors], 
                                      [0 for _ in emio.motors], "displacement")
        MyGui.MoveWindow.setActuatorsLimits(-pi, pi)

    def onAnimateBeginEvent(self, e):
        for leg in self.emio.legs:
            if hasattr(leg.leg, "forceField") and "TetrahedronFEMForceField" in leg.leg.forceField.getValueString():
                leg.leg.TetrahedronFEMForceField.reinit()
            elif hasattr(leg, "deformable") and hasattr(leg.deformable, "forceField") and "BeamHookeLawForceField" in leg.deformable.forceField.getValueString():
                leg.deformable.BeamHookeLawForceField.reinit()

class MyOIMInverseProblemSolver(Sofa.SoftRobotsInverse.QPInverseProblemSolver):
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
        q_a = [m.JointActuator.angle.value for m in self.emio.motors]
        dq_free = np.copy(dfree)
        dq_free[iA] -= q_a

        if self.assembly.done:
            try:
                torques[iA] = self.getTorques(W=W, dq_free=dq_free, iE=iE, iA=iA,
                                               q_s=self.sensor.position.value[0][0:3],
                                               q_t=self.target.position.value[0][0:3],
                                               q_e=self.effector.position.value[0][0:3],
                                               q_a=q_a)
            except Exception:
                if self.lastTorques is not None:
                    torques[iA] = np.copy(self.lastTorques)
        self.lastTorques = np.copy(torques[iA])
        return True

def createScene(rootnode):
    args = getParserArgs()
    # Logic extracted from working QP code
    exercise1 = ("blueleg-direct" in args.legsName)
    exercise2 = (args.legsName[0] == "blueleg")
    exercise3 = (args.legsName[0] == "whiteleg")

    settings, modelling, simulation = addHeader(rootnode, inverse=(not exercise1))
    rootnode.dt, rootnode.gravity = 0.03, [0., -9810., 0.]
    addSolvers(simulation)

    # --- Proxy Model Handling ---
    requested_model = getListFromArgs(args.legsModel)[0]
    vol_variants = ["tetra_linear", "hyper", "nonuniform"]
    proxy_model = "tetra" if requested_model in vol_variants else requested_model
    legs_ym = [parameters.youngModulus * parameters.tetraYMFactor] if proxy_model == "tetra" else [parameters.youngModulus]

    emio = Emio(name="Emio", 
                legsName=["blueleg"] if exercise1 else getListFromArgs(args.legsName),
                legsModel=[proxy_model], 
                legsPositionOnMotor=getListFromArgs(args.legsPositionOnMotor), legsYoungModulus=legs_ym,
                centerPartName=args.centerPartName, centerPartType="rigid", extended=True)
    
    # Physics Injection
    for i, leg in enumerate(emio.legs):
        args.legName = args.legsName[i] if i < len(args.legsName) else args.legsName[0]
        if requested_model == "tetra_linear" and tetra_linear: tetra_linear.create_model(leg, args, parameters, [0,0,0])
        elif requested_model == "hyper" and hyper: hyper.create_model(leg, args, parameters, [0,0,0])
        elif requested_model == "nonuniform" and nonuniform: nonuniform.create_model(leg, args, parameters, [0,0,0])

    simulation.addChild(emio)
    emio.attachCenterPartToLegs()

    effector = emio.effector
    effector.addObject("MechanicalObject", template="Rigid3", position=[0, 0, 0, 0, 0, 0, 1])
    effector.addObject("RigidMapping", index=0)

    # --- Target Setup with Home Position Logic ---
    effectorTarget = None
    if not exercise1:
        effectorTarget = modelling.addChild('Target')
        effectorTarget.addObject('MechanicalObject', template='Rigid3', 
                                 position=[0, -130 if exercise3 else -170, 0, 0, 0, 0, 1], 
                                 showObject=True, showObjectScale=20)

    # --- Sensor & Connection Logic ---
    sensor = emio.effector.getMechanicalState()
    if args.connection and not exercise1:
        try:
            dotTracker = rootnode.addObject(DotTracker(name="DotTracker", root=rootnode, nb_tracker=1, show_video_feed=False))
            sensor = dotTracker.mo
        except RuntimeError:
            Sofa.msg_error(__file__, "Camera not detected")

    # --- Exercise Component Branching ---
    if exercise1:
        emio.addObject(AssemblyController(emio))
        for motor in emio.motors:
            motor.addObject("JointConstraint", name="JointActuator", 
                            minDisplacement=-pi, maxDisplacement=pi, index=0, value=0, valueType="displacement")
        rootnode.addObject(LabGUIExerciseDirect(rootnode, emio))
    elif exercise2 or exercise3:
        rootnode.removeObject(rootnode.ConstraintSolver)
        rootnode.addObject(MyOIMInverseProblemSolver(emio, sensor, effectorTarget, effector, myOIM.getTorques))
        emio.addInverseComponentAndGUI(targetMechaLink=effectorTarget.getMechanicalState().position.linkpath, 
                                       orientationWeight=0, withGUI=True)

    if args.connection:
        emio.addConnectionComponents()
    
    return exercise1, exercise2, exercise3