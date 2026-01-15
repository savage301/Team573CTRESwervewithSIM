from wpimath.geometry import Pose3d, Pose2d
from wpilib import Timer
from photonlibpy import PhotonCamera, PhotonPoseEstimator
import constants
from vision.fieldTagLayout import FieldTagLayout, AprilTagFieldLayout, AprilTagField


class VisionEstimator:
    """
    An estimator (e.g. limelight, photon-vision) that returns a list of robot poses relative to the field.
    """

    def __init__(self):
        self.cam = PhotonCamera('Camera 1')
        self.camPoseEst = PhotonPoseEstimator(
            AprilTagFieldLayout.loadField(AprilTagField.kDefaultField),
            constants.Robot_To_Camera1,
        )

    def get_estimated_robot_pose(self) -> tuple[Pose2d, float] | None:
        """
        Returns the robot's pose relative to the field, estimated by the vision system. Override this method.
        :return: Vision system estimate of robot pose along with the associated timestamp.
        :rtype: list[Pose2d, seconds: float] | None
        """
        camEstPose = None
        for result in self.cam.getAllUnreadResults():
            camEstPose = self.camPoseEst.estimateCoprocMultiTagPose(result)
            if camEstPose is None:
                camEstPose = self.camPoseEst.estimateLowestAmbiguityPose(result)
        if camEstPose is not None:
            return camEstPose.estimatedPose.toPose2d(), camEstPose.timestampSeconds
        else:
            return None