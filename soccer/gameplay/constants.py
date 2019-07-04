import robocup
import math

DegreesToRadians = math.pi / 180.0
RadiansToDegrees = 180.0 / math.pi
OurChipping = (.1, .75)
TheirChipping = (.1, .8)

class Colors:
    White = (255, 255, 255)
    Black = (0, 0, 0)
    Green = (0, 255, 0)
    Red = (255, 0, 0)
    Blue = (0, 0, 255)


class Robot:
    Radius = 0.09
    MaxKickSpeed = 8  # m/s
    ChipClearance = (.1, .6) # min and max distance a chip will go over another robot

    class Dribbler:
        MaxPower = 127

        # "Normal" Dribbler speed to be used for generic ball capture/movement

        StandardPower = 80


class Ball:
    Radius = 0.0215
    Mass = 0.04593  # mass of golf ball (kg)


# Set to the global Field_Dimensions object by the GameplayModule
Field = robocup.Field_Dimensions.CurrentDimensions
