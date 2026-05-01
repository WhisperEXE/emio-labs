import os
import sys
import argparse
import csv # <-- Added CSV import
from math import pi
import xml.etree.ElementTree as ET
from xml.dom import minidom

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

# --- Automated Data Collection Controller ---
class AutoCollector(Sofa.Core.Controller):
    # Added model_name parameter here
    def __init__(self, leg, motor, markers, load, simulation_node, root, model_name):
        Sofa.Core.Controller.__init__(self)
        self.name = "AutoCollector"
        self.listening = True
        self.leg = leg
        self.motor = motor
        self.markers = markers
        self.load = load
        self.root = root
        self.model_name = model_name # Store the model name for the filename

        # Automation Parameters
        self.max_angle = 1.2
        self.intervals = 10
        self.angle_step = self.max_angle / self.intervals
        self.current_interval = 0
        self.started = False

        # Animation state
        self.current_angle = 0.0
        self.speed = 0.01  
        self.settling_frames = 0
        self.max_settling_frames = 150 

        # XML & CSV Setup
        self.xml_root = ET.Element("SimulationData")
        self.csv_data = [] # List to hold our CSV rows
        self.csv_headers = ["Interval", "Angle_rad", 
                            "GM_x", "GM_y", "GM_z", 
                            "M1_x", "M1_y", "M1_z", 
                            "M2_x", "M2_y", "M2_z"]

        # --- Pre-allocate Visual Trails INSIDE the simulation node ---
        self.trail_root = simulation_node.addChild("KinematicsTrails")
        
        hidden_pos = [[0.0, 0.0, 0.0] for _ in range(self.intervals + 1)]
        
        # GreyMass Trail
        node_gm = self.trail_root.addChild("Trail_GM")
        self.trail_gm = node_gm.addObject('MechanicalObject', name='gm_traj', position=hidden_pos, showObject=True, showObjectScale=6.0, showColor=[0.5, 0.5, 0.5, 1])
        node_gm.addObject('UniformMass', totalMass=1.0)        
        node_gm.addObject('FixedConstraint', fixAll=True)      

        # Marker 1 Trail
        node_m1 = self.trail_root.addChild("Trail_M1")
        self.trail_m1 = node_m1.addObject('MechanicalObject', name='m1_traj', position=hidden_pos, showObject=True, showObjectScale=6.0, showColor=[1, 0.2, 0.2, 1])
        node_m1.addObject('UniformMass', totalMass=1.0)
        node_m1.addObject('FixedConstraint', fixAll=True)

        # Marker 2 Trail
        node_m2 = self.trail_root.addChild("Trail_M2")
        self.trail_m2 = node_m2.addObject('MechanicalObject', name='m2_traj', position=hidden_pos, showObject=True, showObjectScale=6.0, showColor=[1, 0.2, 0.2, 1])
        node_m2.addObject('UniformMass', totalMass=1.0)
        node_m2.addObject('FixedConstraint', fixAll=True)

        print("--- Automated Kinematics Collection Ready ---")

    def record_state(self, angle):
        raw_gm = self.load.MechanicalObject.position.value[0][:3]
        raw_m1 = self.markers.MechanicalObject.position.value[0][:3]
        raw_m2 = self.markers.MechanicalObject.position.value[1][:3]

        gm_pos = [float(raw_gm[0]), float(raw_gm[1]), float(raw_gm[2])]
        m1_pos = [float(raw_m1[0]), float(raw_m1[1]), float(raw_m1[2])]
        m2_pos = [float(raw_m2[0]), float(raw_m2[1]), float(raw_m2[2])]

        print(f"\n[Interval {self.current_interval}/10] Motor Angle: {angle:.3f} rad")
        print(f"  GreyMass: [{gm_pos[0]:.2f}, {gm_pos[1]:.2f}, {gm_pos[2]:.2f}]")
        print(f"  Marker 1: [{m1_pos[0]:.2f}, {m1_pos[1]:.2f}, {m1_pos[2]:.2f}]")
        print(f"  Marker 2: [{m2_pos[0]:.2f}, {m2_pos[1]:.2f}, {m2_pos[2]:.2f}]")

        # Save to XML structure
        dp = ET.SubElement(self.xml_root, "DataPoint", angle=f"{angle:.3f}")
        ET.SubElement(dp, "GreyMass", x=str(gm_pos[0]), y=str(gm_pos[1]), z=str(gm_pos[2]))
        ET.SubElement(dp, "Marker1", x=str(m1_pos[0]), y=str(m1_pos[1]), z=str(m1_pos[2]))
        ET.SubElement(dp, "Marker2", x=str(m2_pos[0]), y=str(m2_pos[1]), z=str(m2_pos[2]))

        # --- Save to CSV structure ---
        row = [
            self.current_interval,
            f"{angle:.3f}",
            f"{gm_pos[0]:.3f}", f"{gm_pos[1]:.3f}", f"{gm_pos[2]:.3f}",
            f"{m1_pos[0]:.3f}", f"{m1_pos[1]:.3f}", f"{m1_pos[2]:.3f}",
            f"{m2_pos[0]:.3f}", f"{m2_pos[1]:.3f}", f"{m2_pos[2]:.3f}"
        ]
        self.csv_data.append(row)

        current_gm_trail = [list(p) for p in self.trail_gm.position.value]
        current_m1_trail = [list(p) for p in self.trail_m1.position.value]
        current_m2_trail = [list(p) for p in self.trail_m2.position.value]

        current_gm_trail[self.current_interval] = gm_pos
        current_m1_trail[self.current_interval] = m1_pos
        current_m2_trail[self.current_interval] = m2_pos

        self.trail_gm.position.value = current_gm_trail
        self.trail_m1.position.value = current_m1_trail
        self.trail_m2.position.value = current_m2_trail

    def onAnimateBeginEvent(self, e):
        if hasattr(self.leg.leg, "TetrahedronFEMForceField"):
            self.leg.leg.TetrahedronFEMForceField.reinit()
        elif hasattr(self.leg, "deformable") and hasattr(self.leg.deformable, "BeamHookeLawForceField"):
            self.leg.deformable.BeamHookeLawForceField.reinit()
        elif hasattr(self.leg.leg, "BeamInterpolation"):
            self.leg.leg.BeamInterpolation.lengthY.value = [float(self.leg.width.value)]
            self.leg.leg.BeamInterpolation.lengthZ.value = [float(self.leg.thickness.value)]
            self.leg.leg.BeamInterpolation.defaultYoungModulus.value = [float(self.leg.youngModulus.value)]
            self.leg.leg.BeamInterpolation.defaultPoissonRatio.value = [float(self.leg.poissonRatio.value)]
            self.leg.leg.BeamInterpolation.reinit()

        if not self.started:
            print("\n--- Recording START Phase (0.0 rad) ---")
            self.record_state(0.0) 
            self.current_interval = 1 
            self.started = True
            return

        if self.current_interval > self.intervals:
            return  

        target_angle = self.current_interval * self.angle_step

        if self.current_angle < target_angle:
            self.current_angle += self.speed
            if self.current_angle > target_angle:
                self.current_angle = target_angle
            
            self.motor.JointConstraint.value.value = float(self.current_angle)
            self.settling_frames = 0 
        else:
            self.settling_frames += 1
            if self.settling_frames >= self.max_settling_frames:
                self.record_state(self.current_angle)
                self.current_interval += 1

                if self.current_interval > self.intervals:
                    self.save_data() # Changed from save_xml

    def save_data(self):
        # Create absolute paths targeting the folder this script lives in
        xml_path = os.path.join(current_dir, "kinematics_data.xml")
        csv_path = os.path.join(current_dir, f"{self.model_name}_log.csv")

        # 1. Save XML
        xml_str = minidom.parseString(ET.tostring(self.xml_root)).toprettyxml(indent="  ")
        with open(xml_path, "w") as f:
            f.write(xml_str)
            
        # 2. Save CSV
        with open(csv_path, mode='w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(self.csv_headers)
            writer.writerows(self.csv_data)
            
        print(f"\n>>> SUCCESS: Saved data to {current_dir} <<<")

# --- Scene Creation ---
def createScene(rootnode):
    import myparameters

    parser = argparse.ArgumentParser(prog="Emio Soft Robot Lab")
    parser.add_argument(metavar='legName', type=str, nargs='?', default='blueleg', dest="legName")
    parser.add_argument(metavar='model', type=str, nargs='?', 
                        choices=["cosserat", "beam", "tetra", "tetra_linear", "hyper", "nonuniform"], 
                        default='beam', dest="model")

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

    # --- 1. Load Model ---
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

    # --- 6. Attach Automation Controller ---
    # Passed args.model to the controller to dynamically name the log file
    rootnode.addObject(AutoCollector(leg, motor, markers, load, simulation, rootnode, args.model))
    
    return rootnode