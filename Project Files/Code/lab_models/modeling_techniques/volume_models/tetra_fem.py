import Sofa
from parts.leg import Leg

def create_model(parentNode, args, myparameters, translation):
    # 3D volumetric model using tetrahedral elements
    leg = Leg(name="Leg",
              legName=args.legName,
              positionOnMotor="clockwisedown",
              model="tetra",
              translation=translation,
              rotation=[0, 180, 0],
              youngModulus=myparameters.youngModulus,
              poissonRatio=myparameters.poissonRatio)
    parentNode.addChild(leg)
    return leg