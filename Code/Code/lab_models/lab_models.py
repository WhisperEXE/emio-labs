import os
import sys
import argparse
from math import pi

# --- Path Setup ---
current_dir = os.path.dirname(os.path.realpath(__file__))
if current_dir not in sys.path:
    sys.path.append(current_dir)
project_root = os.path.abspath(os.path.join(current_dir, "../../"))
if project_root not in sys.path:
    sys.path.append(project_root)

import Sofa
import Sofa.Core
import Sofa.ImGui as MyGui

from utils.header import addHeader, addSolvers
from utils import RGBAColor
from splib3.loaders import getLoadingLocation
from splib3.numerics import Vec3, vsub

from parts.motor import Motor
from parts.camera import Camera
from parts.controllers.trackercontroller import DotTracker

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
    print(f"CRITICAL: Model Import Error: {e}")

class LabGUI(Sofa.Core.Controller):
    def __init__(self, leg, motor, markers, model):
        Sofa.Core.Controller.__init__(self)
        self.name = "LabGUI"
        self.root = leg.getRoot()
        self.leg = leg
        self.markers = markers

        MyGui.SimulationState.addData("Motor torque (N.mm)", "M0", motor.JointConstraint.force)
        MyGui.SimulationState.addData("Motor angle (rad)", "M0", motor.JointConstraint.displacement)
        MyGui.MoveWindow.setActuators([motor.JointConstraint.value], [0], "displacement")
        MyGui.MoveWindow.setActuatorsLimits(-pi, pi)

        group = "Error Markers (mm)"
        MyGui.MyRobotWindow.addInformationInGroup("Marker 1", markers.error1, group)
        MyGui.MyRobotWindow.addInformationInGroup("Marker 2", markers.error2, group)

        group = "Mechanical Parameters"
        MyGui.MyRobotWindow.addSettingInGroup("Leg Young modulus", leg.youngModulus, 5e3, 1e5, group)
        MyGui.MyRobotWindow.addSettingInGroup("Leg Poisson ratio", leg.poissonRatio, 0.01, 0.49, group)

        vol_models = ["tetra", "tetra_linear", "hyper", "nonuniform"]
        if model not in vol_models:
            group = "Geometric Parameters (mm)"
            MyGui.MyRobotWindow.addSettingInGroup("Thickness", leg.thickness, 1, 20, group)
            MyGui.MyRobotWindow.addSettingInGroup("Width", leg.width, 1, 20, group)

    def onAnimateBeginEvent(self, e):
        # Update Model Parameters
        if hasattr(self.leg.leg, "TetrahedronFEMForceField"):
            self.leg.leg.TetrahedronFEMForceField.reinit()
        elif hasattr(self.leg, "deformable") and hasattr(self.leg.deformable, "BeamHookeLawForceField"):
            self.leg.deformable.BeamHookeLawForceField.reinit()
        elif hasattr(self.leg.leg, "BeamInterpolation"):
            self.leg.leg.BeamInterpolation.lengthY = [self.leg.width.value]
            self.leg.leg.BeamInterpolation.lengthZ = [self.leg.thickness.value]
            self.leg.leg.BeamInterpolation.defaultYoungModulus = [self.leg.youngModulus.value]
            self.leg.leg.BeamInterpolation.defaultPoissonRatio = [self.leg.poissonRatio.value]
            self.leg.leg.BeamInterpolation.reinit()

        # Error Calculation Logic from old code
        if self.root.getChild("DepthCamera") is not None:
            trackers = self.root.DepthCamera.Trackers.position.value
            markers = self.markers.getMechanicalState().position.value

            if len(trackers) >= 2:
                # Sort trackers to match markers based on position
                if trackers[0][0] > trackers[0][1]:
                    self.markers.error1.value = Vec3(vsub(trackers[0][0:3], markers[1][0:3])).getNorm()
                    self.markers.error2.value = Vec3(vsub(trackers[1][0:3], markers[0][0:3])).getNorm()
                else:
                    self.markers.error1.value = Vec3(vsub(trackers[0][0:3], markers[0][0:3])).getNorm()
                    self.markers.error2.value = Vec3(vsub(trackers[1][0:3], markers[1][0:3])).getNorm()

def createScene(rootnode):
    import myparameters

    parser = argparse.ArgumentParser(prog="Emio Soft Robot Lab")
    parser.add_argument(metavar='legName', type=str, nargs='?', default='blueleg', dest="legName")
    parser.add_argument(metavar='model', type=str, nargs='?', 
                        choices=["cosserat", "beam", "tetra", "tetra_linear", "hyper", "nonuniform"], 
                        default='beam', dest="model")
    parser.add_argument('--no-connection', dest="connection", action='store_false', default=True)

    try:
        args = parser.parse_args(sys.argv[1:])
    except:
        args = parser.parse_args([])

    settings, modelling, simulation = addHeader(rootnode, withCollision=False)
    rootnode.dt = 0.01
    rootnode.gravity = [0., -9810., 0.]
    addSolvers(simulation)

    settings.addObject('RequiredPlugin', name='Sofa.Component.Constraint.Projective')
    translation = [0, 0, 100]

    # --- 1. Load Model with Expanded Selection ---
    if args.model == "beam" and beam:
        leg = beam.create_model(simulation, args, myparameters, translation)
    elif args.model == "cosserat" and cosserat:
        leg = cosserat.create_model(simulation, args, myparameters, translation)
    elif args.model == "tetra" and tetra:
        leg = tetra.create_model(simulation, args, myparameters, translation)
    elif args.model == "tetra_linear" and tetra_linear:
        leg = tetra_linear.create_model(simulation, args, myparameters, translation)
    elif args.model == "hyper" and hyper:
        leg = hyper.create_model(simulation, args, myparameters, translation)
    elif args.model == "nonuniform" and nonuniform:
        leg = nonuniform.create_model(simulation, args, myparameters, translation)
    else:
        Sofa.msg_error("Scene", f"Model '{args.model}' could not be loaded.")
        return

    # --- 2. Setup Motor ---
    motor = Motor(name="Motor", translation=translation, rotation=[0, 180, 0], tempvisurotation=[-90, 180, 0])
    motor.Parts.MotorVisual.activated.value = False
    simulation.addChild(motor)
    leg.attachBase(motor.Parts, 1)

    motor.addObject("JointConstraint", index=0, value=0, 
                    minDisplacement=-pi, maxDisplacement=pi, valueType="displacement")

    # --- 3. Load Environment (Mass Tip) ---
    load = simulation.addChild("Load")
    load.addObject("MechanicalObject", template="Rigid3", position=[[0, -200, 80, 0.707, -0.707, 0., 0.]])
    load.addObject("UniformMass", totalMass=0.034)
    visual = load.addChild("Visual")
    visual.addObject("MeshSTLLoader", 
                     filename=getLoadingLocation("../../data/meshes/centerparts/greymass.stl", __file__),
                     translation=[10, 0, 0], rotation=[0, 90, 0])
    visual.addObject("OglModel", src=visual.MeshSTLLoader.linkpath, color=RGBAColor.grey)
    visual.addObject("RigidMapping")
    leg.attachExtremity(load, 0)

    # --- 4. Decoration & Base ---
    modelling.addChild(Camera())
    base = modelling.addChild("Base")
    base.addObject("MeshSTLLoader", filename=getLoadingLocation("../../data/meshes/base-extended.stl", __file__))
    base.addObject("OglModel", src=base.MeshSTLLoader.linkpath, color=[1, 1, 1, 0.1])

    for i in range(3):
        drum = modelling.addChild(f"Drum{i+1}")
        drum.addObject("MeshSTLLoader", filename=getLoadingLocation("../../data/meshes/legmotorattachbase.stl", __file__),
                       rotation=[0, [90, 180, -90][i], 0], translation=[[-100, 0, 0],[0, 0, -100],[100, 0, 0]][i])
        drum.addObject("OglModel", src=drum.MeshSTLLoader.linkpath, color=[1, 1, 1, 0.1])

    # --- 5. Markers ---
    markers = leg.leg.addChild("Markers")
    markers.addObject("MechanicalObject",
                      position=[[-5, -191, -22.5], [-5, -100, -22.5]] if "blue" in args.legName else [[-5, -191, -22.5], [-22, -110, -22.5]],
                      translation=translation, showObject=True, showObjectScale=7, drawMode=1, showColor=[1, 0, 0, 1])
    
    vol_models = ["tetra", "tetra_linear", "hyper", "nonuniform"]
    markers.addObject("BarycentricMapping" if args.model in vol_models else "SkinningMapping")
    
    markers.addData(name="error1", type="float", value=0)
    markers.addData(name="error2", type="float", value=0)

    # --- 6. Hardware Connection & Tracking ---
    if args.connection:
        try:
            from parts.controllers.motorcontroller import MotorController
            # Initialize MotorController with current displacement
            rootnode.addObject(MotorController([motor.JointConstraint.displacement, None, None, None],
                                            name="MotorController"))

            # Add DotTracker for camera feedback
            tracker = DotTracker(name="DotTracker",
                                 root=rootnode,
                                 configuration="extended",
                                 nb_tracker=2,
                                 show_video_feed=False,
                                 track_colors=True,
                                 comp_point_cloud=False,
                                 scale=1)
            rootnode.addObject(tracker)        
        except (RuntimeError, ImportError) as e:
            Sofa.msg_error(__file__, f"Connection failed: {e}")

    rootnode.addObject(LabGUI(leg, motor, markers, args.model))
    return rootnode