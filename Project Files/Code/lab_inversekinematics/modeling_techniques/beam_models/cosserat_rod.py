import Sofa
from parts.leg import Leg

def create_model(parentNode, args, myparameters, translation):
    # Parametrization via rates of bending, torsion, and elongation
    leg = Leg(name="Leg",
              legName=args.legName,
              model="cosserat",
              translation=translation,
              rotation=[0, 180, 180],
              youngModulus=myparameters.youngModulus,
              poissonRatio=myparameters.poissonRatio,
              thickness=myparameters.thickness,
              width=myparameters.width)
    parentNode.addChild(leg)
    return leg