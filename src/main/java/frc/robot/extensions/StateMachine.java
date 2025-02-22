// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.extensions;

import edu.wpi.first.math.MathUtil;
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
    DEFAULT(0.37, 0, "Red", 0), // field state, default state, no game pieces
    INTAKECORAL(0.520, 0.2, "Orange", 0), // preparing to intake coral
    INTAKEALGAE(0.37, 0, "Yellow", 0), // preparing to intake algae
    STORAGE(0.37, 0, "Green", 0), // intaked, coral or algae in
    L1(7, -0.2, "Blue", 0.1), // scoring coral
    L2(25, -0.2, "Blue", 0.3), // scoring coral
    L3(0, -0.2, "Blue", 0.3), // scoring coral
    PROCESSOR(0.37, 0, "Purple", 0), // scoring algae
    CLIMB(0.37, 0, "Pink", 0), // climbing
    EJECT(0.37, -0.2, "White", 0); // everything out

    public final double elevatorPosition;
    public final double coralIntakeSpeed;
    public final String colorDisplay;
    public final double pivotAngle;

    private BotState(double elevatorPosition, double coralIntakeSpeed, String color, double pivotAngle) {
      this.elevatorPosition = elevatorPosition;
      this.coralIntakeSpeed = coralIntakeSpeed;
      this.colorDisplay = color;
      this.pivotAngle = pivotAngle;
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
