#!/usr/bin/env python3
#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import wpilib
import commands2
import typing
import wpiutil

from robotcontainer import RobotContainer
from vision import vision_sim
from telemetry import Telemetry

from config import Cameras
from wpimath.geometry import Pose2d, Rotation2d

class MyRobot(commands2.TimedCommandRobot):
    """
    Command v2 robots are encouraged to inherit from TimedCommandRobot, which
    has an implementation of robotPeriodic which runs the scheduler for you
    """

    autonomousCommand: typing.Optional[commands2.Command] = None

    def robotInit(self) -> None:
        """
        This function is run when the robot is first started up and should be used for any
        initialization code.
        """

        # Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        # autonomous chooser on the dashboard.
        self.container = RobotContainer()
        self.container.drivetrain.reset_pose(Pose2d(5,5,Rotation2d(0)))

        # Init Simulation specifics
        if wpilib.RobotBase.isSimulation():
            self.visionSim = vision_sim.photonvision_sim_setup() #Setup sim vision system

    def robotPeriodic(self) -> None:
        """This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
        that you want ran during disabled, autonomous, teleoperated and test.

        This runs after the mode specific periodic functions, but before LiveWindow and
        SmartDashboard integrated updating."""

        # Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        # commands, running already-scheduled commands, removing finished or interrupted commands,
        # and running subsystem periodic() methods.  This must be called from the robot's periodic
        # block in order for anything in the Command-based framework to work.

        
        if wpilib.RobotBase.isSimulation():
            self.visionSim.update(self.container.drivetrain.get_state().pose)
            self.cameravis = self.visionSim.getDebugField()
        
        #print("Current Angle: ", self.container.drivetrain.get_state().pose.rotation().degrees())
        self.add_vision_to_pose_esimate()
        commands2.CommandScheduler.getInstance().run()

    def disabledInit(self) -> None:
        """This function is called once each time the robot enters Disabled mode."""
        pass

    def disabledPeriodic(self) -> None:
        """This function is called periodically when disabled"""
        pass

    def autonomousInit(self) -> None:
        """This autonomous runs the autonomous command selected by your RobotContainer class."""
        self.autonomousCommand = self.container.getAutonomousCommand()

        if self.autonomousCommand:
            self.autonomousCommand.schedule()

    def autonomousPeriodic(self) -> None:
        """This function is called periodically during autonomous"""
        pass

    def teleopInit(self) -> None:
        # This makes sure that the autonomous stops running when
        # teleop starts running. If you want the autonomous to
        # continue until interrupted by another command, remove
        # this line or comment it out.
        if self.autonomousCommand:
            self.autonomousCommand.cancel()

    def teleopPeriodic(self) -> None:
        """This function is called periodically during operator control"""
        #pass
        if self.container._joystick.y().getAsBoolean():
            self.container.talonfx.set_control(self.container.motion_magic.with_position(10).with_slot(0))
        else:
            self.container.talonfx.set_control(self.container.motion_magic.with_position(0).with_slot(0))
        self.container._field1_pub.set(self.container.talonfx.get_position().value_as_double)
        self.container._field2_pub.set(self.container.motion_magic.position)

    def testInit(self) -> None:
        # Cancels all running commands at the start of test mode
        commands2.CommandScheduler.getInstance().cancelAll()

    def add_vision_to_pose_esimate(self):
        current_pose = self.container.drivetrain.get_state().pose
        vision_ests = self.container._vision_est.get_estimated_robot_pose(current_pose)
        if vision_ests is not None:
            for vision_est in vision_ests:
                esti_pose = vision_est[0]
                if esti_pose is not None:
                    relative = esti_pose.relativeTo(current_pose)
                    dist = (relative.X()**2 + relative.Y()**2)**(1/2)
                    if  dist < 1:
                        self.container.drivetrain.add_vision_measurement(vision_est[0],vision_est[1],vision_est[2])
                        #print("Odo Pose: ", self.container.drivetrain.get_state().pose, "Vision Est Pose", vision_est[0])
        