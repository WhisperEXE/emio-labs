from parts.leg import Leg


def create_model(parentNode, args, myparameters, translation):
    plugin_name = "Sofa.Component.SolidMechanics.FEM.NonUniform"

    leg = Leg(
        name="Leg",
        legName=args.legName,
        model="tetra",
        translation=translation,
        rotation=[0, 180, 0],
        youngModulus=myparameters.youngModulus,
        poissonRatio=myparameters.poissonRatio,
    )

    try:
        leg.leg.addObject("RequiredPlugin", name=plugin_name)
    except Exception:
        print(f"Non-Uniform plugin not found for {plugin_name}. Falling back to default.")

    parentNode.addChild(leg)
    return leg
