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
        position: 0,
    ) -> None:
        super().__init__()
        
        self.app = app
        self.addRequirements(app)
        self.position = position

      def execute(self):
        self.app.setElevatorPosition(self.position)

      def isFinished(self):
         if abs(Elevator.getElevatorPosition(self.app) - self.position) < .5:
            return True
        
      def end(self, interrupted=False) -> None:
        #This is run when the command is finished.
        #Stops elevator motors when the command is finished.
        self.app.stopElevator()


