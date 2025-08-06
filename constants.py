import math

from wpimath.geometry import Pose3d, Rotation3d, Transform3d, Pose2d, Translation2d, Rotation2d, Translation3d

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