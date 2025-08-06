from wpimath.geometry import Pose3d, Pose2d
from wpilib import Timer


class VisionEstimator:
    """
    An estimator (e.g. limelight, photon-vision) that returns a list of robot poses relative to the field.
    """

    def __init__(self):
        self.pose = Pose2d(0,0,0)
        self.timestamp = 0
        self.trust = 0
        self.std = (0,0,0)

    def get_estimated_robot_pose(self) -> list[Pose3d, float] | None:
        """
        Returns the robot's pose relative to the field, estimated by the vision system. Override this method.
        :return: Vision system estimate of robot pose along with the associated timestamp.
        :rtype: list[Pose3d, seconds: float] | None
        """
        raise NotImplementedError
