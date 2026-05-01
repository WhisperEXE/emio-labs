import Sofa
import os
from parts.leg import Leg

def create_model(parentNode, args, myparameters, translation):
    plugin_path = "/home/think/emio-labs/external_plugins"
    plugin_name = "Sofa.Component.SolidMechanics.FEM.HyperElastic"

    # Resolve 'SceneCheckMissingRequiredPlugin' warning
    parentNode.addObject('RequiredPlugin', name='Sofa.Component.SceneUtility')

    if os.path.exists(plugin_path):
        parentNode.addObject('AddPluginRepository', path=plugin_path)

    # 3D Volume base
    leg = Leg(name="Leg",
              legName=args.legName,
              model="tetra", 
              translation=translation,
              rotation=[0, 180, 180],
              youngModulus=myparameters.youngModulus,
              poissonRatio=myparameters.poissonRatio)
    
    # Safe loading without using Sofa.Helper.findPlugin
    try:
        leg.leg.addObject('RequiredPlugin', name=plugin_name)
        # This handles non-linear material stretching for silicone
    except Exception:
        print(f"CRITICAL: {plugin_name} not found in {plugin_path}. Falling back to default linear physics.")
    
    parentNode.addChild(leg)
    return leg