// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.StateMachineConstant;
import frc.robot.extensions.StateMachine;
import frc.robot.extensions.StateMachine.BotState;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.Elevator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class cm_MoveAndEject extends ParallelCommandGroup {
  /** Creates a new cm_MoveAndEject. */
  // TODO: Change to elevator velocity control request
  public cm_MoveAndEject(Elevator elevator, CoralIntake coralIntake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      StateMachine.setGlobalState(BotState.INTAKEALGAE),
      elevator.cm_moveElevator(0.3),
      coralIntake.cm_intakeCoral(StateMachineConstant.botState.coralIntakeSpeed)
    );
  }
}
