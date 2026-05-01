from parts.leg import Leg


def create_model(parentNode, args, myparameters, translation):
    leg = Leg(
        name="Leg",
        legName=args.legName,
        positionOnMotor="clockwisedown",
        model="tetra",
        translation=translation,
        rotation=[0, 180, 0],
        youngModulus=myparameters.youngModulus,
        poissonRatio=myparameters.poissonRatio,
    )

    # Improve stability when the external direct solver plugin is available.
    try:
        leg.leg.addObject("RequiredPlugin", name="Sofa.Component.LinearSolver.Direct")
    except Exception:
        pass

    parentNode.addChild(leg)
    return leg
