import wpilib
import time
from wpimath.units import feetToMeters, inchesToMeters
from wpimath.geometry import Pose2d
from photonlibpy.photonCamera import PhotonCamera #VisionLEDMode
from vision.fieldTagLayout import FieldTagLayout
from vision.vision_estimator import VisionEstimator
from wpimath.geometry import Pose3d, Translation3d, Rotation3d, Transform3d, Transform2d
import math
from wpilib import Timer
import config

# Describes one on-field pose estimate from the a camera at a specific time.
class CameraPoseObservation:
    def __init__(self, time, estFieldPose, trustworthiness=1.0):
        self.time = time
        self.estFieldPose = estFieldPose
        self.trustworthiness = trustworthiness  # TODO - not used yet

# Wrappers photonvision to:
# 1 - resolve issues with target ambiguity (two possible poses for each observation)
# 2 - Convert pose estimates to the field
# 3 - Handle recording latency of when the image was actually seen
class WrapperedPhotonCamera:
    def __init__(self, camName, robotToCam):
        #setVersionCheckEnabled(False)
        self.cam_name = camName
        self.cam = PhotonCamera(camName)
        #print("Camera: ", self.cam_name)
        #self.disconFault = Fault(f"Camera {camName} not sending data")
        self.timeoutSec = 1.0
        self.poseEstimates = []
        self.robotToCam : Transform3d = robotToCam
        #print(type(self.robotToCam))
        
    
    def update(self, prevEstPose:Pose2d):
        self.poseEstimates = []
        self.targetEstimates: list[str, int, Transform3d] = []

        if(not self.cam.isConnected()):
            # Faulted - no estimates, just return.
            #self.disconFault.setFaulted()
            return

        # Grab whatever the camera last reported for observations in a camera frame
        # Note: Results simply report "I processed a frame". There may be 0 or more targets seen in a frame
        res = self.cam.getLatestResult()

        # MiniHack - results also have a more accurate "getTimestamp()", but this is
        # broken in photonvision 2.4.2. Hack with the non-broken latency calcualtion
        latency = res.getLatencyMillis()
        obsTime = time.time() - latency/1000
        

        # Update our disconnected fault since we have something from the camera
        #self.disconFault.setNoFault()

        # Process each target.
        # Each target has multiple solutions for where you could have been at on the field
        # when you observed it
        # (https://docs.wpilib.org/en/stable/docs/software/vision-processing/
        # apriltag/apriltag-intro.html#d-to-3d-ambiguity)
        # We want to select the best possible pose per target
        # We should also filter out targets that are too far away, and poses which
        # don't make sense.
        for target in res.getTargets():
            # Transform both poses to on-field poses
            tgtID = target.getFiducialId()
            #print(tgtID)
            if tgtID >= 0:
                # Only handle valid ID's
                tagFieldPose = FieldTagLayout().lookup(tgtID)
                #print(tagFieldPose)
                if tagFieldPose is not None:
                    # Only handle known tags
                    poseCandidates:list[Pose2d,float] = []
        
                    self.targetEstimates.append([tgtID, target.getBestCameraToTarget()])
                    poseCandidates.append([
                        self._toFieldPose(tagFieldPose, target.getBestCameraToTarget()),
                        self._toFieldPose(tagFieldPose, target.getBestCameraToTarget()).relativeTo(tagFieldPose.toPose2d()).translation().norm()] ####################################################################################
                    )

                    filteredCandidates:list[Pose2d,float] = []
                    for candidate in poseCandidates:
                        onField = self._poseIsOnField(candidate[0])
                        # Add other filter conditions here
                        if onField:
                            filteredCandidates.append([candidate[0],candidate[1]])

                    # Pick the candidate closest to the last estimate
                    bestCandidate:(list[Pose2d,float]|None) = None
                    bestCandidateDist = 99999999.0
                    for candidate in filteredCandidates:
                        delta = (candidate[0] - prevEstPose).translation().norm()
                        if delta < bestCandidateDist:
                            # This candidate is better, use it
                            bestCandidate = candidate[0]
                            bestCandidateDisttoTag = candidate[1]
                            bestCandidateDist = delta
                    # Finally, add our best candidate the list of pose observations
                    if bestCandidate is not None:
                        #print(bestCandidate)
                        #Here is were we put in the trust of the target based on distance from it.
                        #Trust is a value from 0 to 1. 1 is the most trustworthy, 0 is the least.
                        trust = config.vision_settings.tursttoDist_ratio*bestCandidateDisttoTag + config.vision_settings.tursttoDist_offset
                        if trust < 0:
                            trust = 0
                        if trust > 1:
                            trust = 1
                        self.poseEstimates.append(
                            CameraPoseObservation(obsTime, bestCandidate,trust)
                            )

    def getPoseEstimates(self):
        return self.poseEstimates
    
    def getTargetEstimates(self):
        return self.targetEstimates

    def _toFieldPose(self, tgtPose: Pose3d, camToTarget: Transform3d):
        camPose = tgtPose.transformBy(camToTarget.inverse())
        return camPose.transformBy(self.robotToCam.inverse()).toPose2d()

    def getDistanceTwo(target:Translation3d) -> float:
        output = float(math.sqrt((target.X())**2 + (target.Y())**2 + (target.Z())**2))
        #print("Distance to Target: ", output)
        return output

    # Returns true of a pose is on the field, false if it's outside of the field perimieter
    def _poseIsOnField(self, pose: Pose2d):
        trans = pose.translation()
        x = trans.X()
        y = trans.Y()
        inY = 0.0 <= y <= feetToMeters(27.0)
        inX = 0.0 <= x <= feetToMeters(54.0)
        return inX and inY
    
class PhotonVisionController(VisionEstimator):
    def __init__(self, photovisionCameracamera_list: list[WrapperedPhotonCamera]):
        super().__init__()
        self.cameras = photovisionCameracamera_list

    def get_estimated_robot_pose(self,prevPose:Pose2d):
        """
        Returns the robot's pose relative to the field, estimated by the photonvision.
        :return: Camera estimate of robot pose.
        :rtype: Pose3d | None
        """
        vision_estimator_list = []
        self.pose = None
        self.timestamp = 0
        self.trust = 0
        self.std = (0,0,0)
        
        for camera in self.cameras:
            camera.update(prevPose)
            est_poses = camera.getPoseEstimates()
            for est_pose in est_poses:
                lastpose = est_pose.estFieldPose
                lasttime= est_pose.time
                lasttrust= est_pose.trustworthiness
                if lasttrust > self.trust:
                    self.pose = lastpose
                    self.timestamp = lasttime
                    self.trust = lasttrust    

                    stdX = (1-self.trust) * config.vision_settings.stdXtrust_ratio + config.vision_settings.stdXtrust_min
                    stdY = (1-self.trust) * config.vision_settings.stdYtrust_ratio + config.vision_settings.stdYtrust_min
                    stdT = (1-self.trust) * config.vision_settings.stdRtrust_ratio + config.vision_settings.stdRtrust_min

                    #Look into using trust vs std dev
                    #print("Trust: ", self.trust)
                    self.std = (stdX,stdY,stdT)  

            vision_estimator_list.append((self.pose,self.timestamp,self.std))
            
        return vision_estimator_list
    
    def get_target_list(self) -> list[str, int, Transform3d] | None:
        # Returns list of seen targets and their Transform from the camera. Can be used for auto alignment. 
        # Transform3d Y-axis is the left right relative to camera
        target_list = []
        for camera in self.cameras:
            est_target = camera.getTargetEstimates()
            #print(est_target)
            camername = camera.cam_name
            #print(camername)
            if est_target is not []:
                try:
                    lastAprilTag = est_target[-1][0] 
                    lasttarget = est_target[-1][1]
                    target_list.append([camername,lastAprilTag,lasttarget]) # LastTarget = transform3D 
                except:
                    pass
        return target_list if target_list else None
    
