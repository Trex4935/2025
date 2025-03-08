// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.extensions.StateMachine;
import frc.robot.extensions.StateMachine.BotState;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.Elevator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class cm_FullSequence extends SequentialCommandGroup {
  /** Creates a new cm_FullSequence. */
  public cm_FullSequence(BotState botState, Elevator elevator, CoralIntake coralIntake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        StateMachine.setGlobalState(botState),
        new cm_SetElevatorPosition(elevator).withTimeout(10),
        new cm_SetPivotAngle(coralIntake).withTimeout(3),
        new cm_SetCoralIntake(coralIntake).withTimeout(3));
  }
}
