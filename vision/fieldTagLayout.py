from robotpy_apriltag import AprilTagField, AprilTagFieldLayout
from vision.singleton import Singleton


class FieldTagLayout(metaclass=Singleton):
    def __init__(self):
        #Field tags
        self.fieldTags = AprilTagFieldLayout.loadField(AprilTagField.k2025ReefscapeWelded)
        #Shop tags
        #self.fieldTags = constants.Shop_AprilTagLayout

    def lookup(self, tagId):
        return self.fieldTags.getTagPose(tagId)