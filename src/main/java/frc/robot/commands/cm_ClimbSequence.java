package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Climber;

public class cm_ClimbSequence extends SequentialCommandGroup {

  public cm_ClimbSequence(Climber climber, double setPosition, double liftPosition) {
    addCommands(
        // Deploys solenoid
        climber.cm_solenoidToggle(),

        // Sets position of motor using motion magic, need to tune constants
        new cm_SetClimberPosition(climber, setPosition).withTimeout(5),

        // Reverses climber motor to a lift position
        new cm_SetClimberPosition(climber, liftPosition).withTimeout(5));
  }
}
