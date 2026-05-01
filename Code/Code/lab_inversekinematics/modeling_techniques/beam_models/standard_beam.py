import Sofa
from parts.leg import Leg

def create_model(parentNode, args, myparameters, translation):
    leg = Leg(name="Leg",
              legName=args.legName,
              positionOnMotor="clockwisedown",
              model="beam",
              translation=translation,
              rotation=[0, 180, 0],
              youngModulus=myparameters.youngModulus,
              poissonRatio=myparameters.poissonRatio,
              thickness=myparameters.thickness,
              width=myparameters.width)
    parentNode.addChild(leg)
    return leg