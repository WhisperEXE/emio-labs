import Sofa
import os
from parts.leg import Leg

def create_model(parentNode, args, myparameters, translation):
    plugin_path = "/home/think/emio-labs/external_plugins"
    plugin_name = "Sofa.Component.SolidMechanics.FEM.NonUniform"

    # Required for AddPluginRepository to work without warnings
    parentNode.addObject('RequiredPlugin', name='Sofa.Component.SceneUtility')

    if os.path.exists(plugin_path):
        parentNode.addObject('AddPluginRepository', path=plugin_path)

    leg = Leg(name="Leg",
              legName=args.legName,
              model="tetra", 
              translation=translation,
              rotation=[0, 180, 180],
              youngModulus=myparameters.youngModulus,
              poissonRatio=myparameters.poissonRatio)
    
    # Safe loading for your specific SOFA version
    try:
        leg.leg.addObject('RequiredPlugin', name=plugin_name)
        # Non-uniform models allow for varying stiffness across the mesh
    except Exception:
        print(f"Non-Uniform plugin not found in {plugin_path}. Falling back to default.")
    
    parentNode.addChild(leg)
    return leg