import math

from wpimath.geometry import Pose3d, Rotation3d, Transform3d, Pose2d, Translation2d, Rotation2d, Translation3d
from pathplannerlib.auto import PathConstraints

inches_to_meters = 0.0254
#----------------------------- Camera Locations ---------------------------
Robot_To_Camera1 = Transform3d(
    Translation3d(
       7*inches_to_meters, 9.5*inches_to_meters, 12.5*inches_to_meters  # X  # Y  # Z
    ),
        Rotation3d(0.0, 0.0, math.radians(-15)),  # Roll  # Pitch  # Yaw
    )

Robot_To_Camera2 = Transform3d(
    Translation3d(
       7*inches_to_meters, -8.5*inches_to_meters, 12.5*inches_to_meters  # X  # Y  # Z
    ),
        Rotation3d(0.0, 0.0, math.radians(15)),  # Roll  # Pitch  # Yaw
    )

Robot_To_Camera3 = Transform3d(
    Translation3d(
       0*inches_to_meters, -7*inches_to_meters, 24*inches_to_meters  # X  # Y  # Z
    ),
        Rotation3d(math.radians(0), math.radians(-1*(90-56)), math.radians(180-15)),  # Roll  # Pitch  # Yaw
    )

# ---------------------------------------------------------

#Drivetrain Constants
class DrivetrainConstants:
    MaxVelocity = 4.5 #meters per second
    MaxAcceleration = 3 #meters per second squared
    MaxAngularVelocity = math.pi #radians per second
    MaxAngularAcceleration = math.pi #radians per second squared

    constraints = PathConstraints(MaxVelocity,MaxAcceleration,MaxAngularVelocity,MaxAngularAcceleration)

class HPStations:

    #Offeset from wall at HP for robot to stop at
    HPOffset = 0.01 # in meters
    # HP Positions
    blue_right_HP = Pose2d(0.851, 0.654, Rotation2d((54 * math.pi) / 180))
    postOffset_BlueR_HP = Pose2d(blue_right_HP.X() + math.cos(blue_right_HP.rotation().radians()) * HPOffset, blue_right_HP.Y() + math.sin(blue_right_HP.rotation().radians()), blue_right_HP.rotation())
    blue_left_HP =  Pose2d(0.851, 7.404, Rotation2d((306 * math.pi) / 180))
    postOffset_BlueL_HP = Pose2d(blue_left_HP.X() + math.cos(blue_left_HP.rotation().radians()) * HPOffset, blue_left_HP.Y() + math.sin(blue_left_HP.rotation().radians()), blue_left_HP.rotation())
    red_right_HP = Pose2d(16.711, 7.404, Rotation2d((234 * math.pi) / 180))
    PostOffset_RedR_HP = Pose2d(red_right_HP.X() + math.cos(red_right_HP.rotation().radians()) * HPOffset, red_right_HP.Y() + math.sin(red_right_HP.rotation().radians()), red_right_HP.rotation())
    red_left_HP = Pose2d(16.711, 0.654, Rotation2d((126 * math.pi) / 180))
    PostOffset_RedL_HP = Pose2d(red_left_HP.X() + math.cos(red_right_HP.rotation().radians()) * HPOffset, red_left_HP.Y() + math.sin(red_left_HP.rotation().radians()), red_left_HP.rotation())

    humanPlayersPos = [
        postOffset_BlueL_HP, postOffset_BlueR_HP, PostOffset_RedR_HP, PostOffset_RedL_HP
]

class ReefPositions:

    class BlueReef:
        BlueReefPos1 = Pose2d(inches_to_meters*(144.00), inches_to_meters*(158.50), Rotation2d(math.radians(0))) #april tasg 18
        BlueReefPos2 = Pose2d(inches_to_meters*(160.39), inches_to_meters*(130.17), Rotation2d(math.radians(60))) #april tag 17
        BlueReefPos3 = Pose2d(inches_to_meters*(193.10), inches_to_meters*(130.17), Rotation2d(math.radians(120)))  #april tag 22
        BlueReefPos4 = Pose2d(inches_to_meters*(209.49), inches_to_meters*(158.50), Rotation2d(math.radians(180)))  #april tag 21
        BlueReefPos5 = Pose2d(inches_to_meters*(193.10), inches_to_meters*(186.83), Rotation2d(math.radians(240)))  #arpil tag 20
        BlueReefPos6 = Pose2d(inches_to_meters*(160.39), inches_to_meters*(186.83), Rotation2d(math.radians(300))) #april tag 19

        BlueReefList = [
            BlueReefPos1, BlueReefPos2, BlueReefPos3, BlueReefPos4, BlueReefPos5, BlueReefPos6
        ]

    class RedReef:
        RedReefPos1 = Pose2d(Translation2d(inches_to_meters*(546.87), inches_to_meters*(158.50)), Rotation2d(math.radians(180))) #april tag 7
        RedReefPos2 = Pose2d(Translation2d(inches_to_meters*(530.49), inches_to_meters*(130.17)), Rotation2d(math.radians(120))) #april tag 6
        RedReefPos3 = Pose2d(Translation2d(inches_to_meters*(497.77), inches_to_meters*(130.17)), Rotation2d(math.radians(60)))  #april tag 11
        RedReefPos4 = Pose2d(Translation2d(inches_to_meters*(481.39), inches_to_meters*(158.50)), Rotation2d(math.radians(0))) #april tag 10
        RedReefPos5 = Pose2d(Translation2d(inches_to_meters*(497.77), inches_to_meters*(186.83)), Rotation2d(math.radians(-60))) #april tag 9
        RedReefPos6 = Pose2d(Translation2d(inches_to_meters*(530.49), inches_to_meters*(186.83)), Rotation2d(math.radians(-120))) #april tag 8

        RedReefList = [
            RedReefPos1, RedReefPos2, RedReefPos3, RedReefPos4, RedReefPos5, RedReefPos6
        ]