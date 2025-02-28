// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.extensions;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.StateMachineConstant;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.Elevator;

/** Add your docs here. */
public class StateMachine {

  /**
   * An enum used to store current bot state, as well as target positions for both pivot and wrist
   * in that state (in degrees).
   *
   * @apiNote (pivotDegrees, wristDegrees)
   * @apiNote Wrist degrees are from a somewhat horizontal position, this is because of needing
   *     gravity feedforward.
   */
  public enum BotState {
    DEFAULT(0.37, 0, Color.kRed), // field state, default state, no game pieces
    INTAKECORAL(0.520, 5, Color.kOrange), // preparing to intake coral
    INTAKEALGAE(0.37, 0, Color.kYellow), // preparing to intake algae
    STORAGE(0.37, 0, Color.kGreen), // intaked, coral or algae in
    L1(7, -5, Color.kBlue), // scoring coral
    L2(25, -5, Color.kBlue), // scoring coral
    L3(0, -5, Color.kBlue), // scoring coral
    PROCESSOR(0.37, 0, Color.kPurple), // scoring algae
    CLIMB(0.37, 0, Color.kPink), // climbing
    EJECT(0.37, -5, Color.kWhite); // everything out

    public final double elevatorPosition;
    public final double coralIntakePosition;
    public final Color colorDisplay;

    private BotState(double elevatorPosition, double coralIntakePosition, Color color) {
      this.elevatorPosition = elevatorPosition;
      this.coralIntakePosition = coralIntakePosition;
      this.colorDisplay = color;
    }
  }

  public static Command setGlobalState(BotState state) {
    return Commands.runOnce(() -> Constants.StateMachineConstant.setState(state));
  }

  public static Command scoringSequence(Elevator elevator, CoralIntake coralIntake) {
    return new SequentialCommandGroup(
        elevator
            .cm_setElevatorPosition(StateMachineConstant.botState.elevatorPosition)
            .until(
                () ->
                    MathUtil.isNear(
                        StateMachineConstant.botState.elevatorPosition,
                        elevator.leftElevatorMotor.getPosition().getValueAsDouble(),
                        1)),
        elevator.cm_setElevatorPosition(15)); // Temp command to test sequencing
    // coralIntake.cm_intakeCoral(botState.coralIntakeSpeed));
  }
}
