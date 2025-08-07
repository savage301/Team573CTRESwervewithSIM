import typing
import commands2
import wpilib

from oi.keymap import Keymap

# from commands.wrist import Wrist

from subsystems.elevator import Elevator     


class setPosition(commands2.Command):
     def __init__(
        # This defines what subsystem this command is for, so it can be used in the command scheduler.
        self, 
        app: Elevator,
    ) -> None:
        super().__init__()
        
        self.app = app
        self.addRequirements(app)

     def execute(self):
        #print("Excute")
        self.app.setElevatorPosition(10)
        
     def end(self, interrupted=False) -> None:
        #This is run when the command is finished.
        #Stops elevator motors when the command is finished.
        self.app.setElevatorPosition(0)


