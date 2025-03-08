// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.Constants.StateMachineConstant;
import frc.robot.extensions.StateMachine;
import frc.robot.extensions.StateMachine.BotState;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.Elevator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class cm_AlgaeRemoval extends ParallelDeadlineGroup {
  /** Creates a new cm_AlgaeRemoval. */
  public cm_AlgaeRemoval(Elevator elevator, CoralIntake coralIntake) {
    // Add the deadline command in the super() call. Add other commands using
    // addCommands().
    super(new cm_SetElevatorPosition(elevator));
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        StateMachine.setGlobalState(BotState.REMOVEALGAE),
        coralIntake.cm_intakeCoral(StateMachineConstant.botState.coralIntakeSpeed));
  }
}
