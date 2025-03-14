package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.Climber;

public class cm_ClimbStart extends SequentialCommandGroup {

    public cm_ClimbStart(Climber climber, double setPosition, double liftPosition){
        addCommands(
            // Enables solenoid for 1 second
            climber.cm_solenoidToggle(),
            // Sets position of motor using motion magic, need to tune constants
            climber.cm_climberMovement(setPosition)
            .until(() -> MathUtil.isNear(setPosition, climber.climberMotor.getPosition().getValueAsDouble(),0.5)),
            // Reverses climber motor back to position
            climber.cm_climberMovement(liftPosition).until(() -> MathUtil.isNear(liftPosition, climber.climberMotor.getPosition().getValueAsDouble(), 0.5)));

    }
}
