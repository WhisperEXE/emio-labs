import Sofa
import os
from parts.leg import Leg

def create_model(parentNode, args, myparameters, translation):
    plugin_path = "/home/think/emio-labs/external_plugins"
    
    # Required for AddPluginRepository
    parentNode.addObject('RequiredPlugin', name='Sofa.Component.SceneUtility')
    
    if os.path.exists(plugin_path):
        parentNode.addObject('AddPluginRepository', path=plugin_path)

    leg = Leg(name="Leg",
              legName=args.legName,
              positionOnMotor="clockwisedown",
              model="tetra",
              translation=translation,
              rotation=[0, 180, 0],
              youngModulus=myparameters.youngModulus,
              poissonRatio=myparameters.poissonRatio)
    
    # Use a try/except block to safely attempt loading the grafted plugin
    try:
        leg.leg.addObject('RequiredPlugin', name="Sofa.Component.LinearSolver.Direct")
        # Direct solvers improve stability for static equilibrium tasks
    except Exception:
        print("Direct Solver plugin not found in external_plugins. Using default solvers.")

    parentNode.addChild(leg)
    return leg