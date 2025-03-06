// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.extensions.StateMachine;
import frc.robot.extensions.StateMachine.BotState;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.Elevator;

public final class Autos {

  /** Example static factory for an autonomous command. */
  /* public static Command exampleAuto(ExampleSubsystem subsystem) {
    return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));
  } */
  public static final Elevator elevator = RobotContainer.m_elevator;

  public static final CoralIntake coralIntake = RobotContainer.m_coralIntake;

  // Add NamedCommands here
  public static Command L1 =
      Commands.sequence(new cm_FullSequence(BotState.L1, elevator, coralIntake))
          .andThen(new Shooting(elevator, coralIntake));

  public static Command L2 =
      Commands.sequence(new cm_FullSequence(BotState.L2, elevator, coralIntake))
          .andThen(new Shooting(elevator, coralIntake));

  public static Command L3 =
      Commands.sequence(new cm_FullSequence(BotState.L3, elevator, coralIntake))
          .andThen(new Shooting(elevator, coralIntake));
  // public static Command L4 =
  //  Commands.runOnce(StateMachine.setGlobalState(BotState.).andThen());
  public static Command Default =
      Commands.sequence(new cm_FullSequence(BotState.DEFAULT, elevator, coralIntake))
          .andThen(new Shooting(elevator, coralIntake));
  public static Command runIntake =
      Commands.sequence(
          StateMachine.setGlobalState(BotState.INTAKECORAL)
              .andThen(new cm_SetCoralIntake(coralIntake)));

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
