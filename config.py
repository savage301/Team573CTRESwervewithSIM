from vision.photon_camera import WrapperedPhotonCamera, PhotonVisionController
import constants
# ------------ Vision Settings --------------------

class vision_settings:
    #These are the ratio to get std dev for the vision system based on trust where 1 is the most trustworthy and 0 is the least
    #So eahc is trust_ratio * (1-trust) + trust_min
    Xtrust_ratio = 0.7
    Ytrust_ratio = 0.7
    Rtrust_ratio = 0.7
    #This is the minumum value for the std dev for any vision mesaurement
    Xtrust_min = 0.1
    Ytrust_min = 0.1
    Rtrust_min = 0.1

    #These are settings for trust based on distance from the target
    #The trust is a value from 0 to 1 where 1 is the most trustworthy and 0 is the least
    #The trust is calculated as -1/10*distance + 1 and is bound between 0 and 1
    tursttoDist_ratio = -1/10 #This should always be negative for example -1/10 means at 10 meters the trust is 0
    tursttoDist_offset = 1 #This should always be 1

class Cameras:
    vision_controller = PhotonVisionController([WrapperedPhotonCamera("Camera1",constants.Robot_To_Camera1),WrapperedPhotonCamera("Camera2",constants.Robot_To_Camera2),WrapperedPhotonCamera("Camera3",constants.Robot_To_Camera3)])