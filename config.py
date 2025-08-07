from vision.photon_camera import WrapperedPhotonCamera, PhotonVisionController
import constants
from utils.utils import inches_to_meters
from phoenix6 import configs, controls

# ------------ Elevator Settings -----------------
class Elevator:
    MinLength = inches_to_meters(30) # Elevator height all the way down.
    Rot_to_Dist = 5*0.0254 # Number of Rotations to go 1 m
    cfg = configs.TalonFXConfiguration()
     # Configure gear ratio
    fdb = cfg.feedback
    fdb.sensor_to_mechanism_ratio = 1 # 12.8 rotor rotations per mechanism rotation

    # Configure Motion Magic
    mm = cfg.motion_magic
    mm.motion_magic_cruise_velocity = 2 # 5 (mechanism) rotations per second cruise
    mm.motion_magic_acceleration = 5 # Take approximately 0.5 seconds to reach max vel
    # Take apprximately 0.1 seconds to reach max accel
    mm.motion_magic_jerk = 100

    slot0 = cfg.slot0
    slot0.k_s = 0.25 # Add 0.25 V output to overcome static friction
    slot0.k_v = 0.12 # A velocity target of 1 rps results in 0.12 V output
    slot0.k_a = 0.01 # An acceleration of 1 rps/s requires 0.01 V output
    slot0.k_p = 10 # A position error of 0.2 rotations results in 12 V output
    slot0.k_i = 0 # No output for integrated error
    slot0.k_d = 0.5 # A velocity error of 1 rps results in 0.5 V output


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