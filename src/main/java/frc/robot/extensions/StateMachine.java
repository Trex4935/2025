// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.extensions;

import edu.wpi.first.wpilibj.util.Color;

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
    DEFAULT(0, 0, Color.kRed), // field state, default state, no game pieces
    INTAKECORAL(0, 0.2, Color.kOrange), // preparing to intake coral
    INTAKEALGAE(0, 0, Color.kYellow), // preparing to intake algae
    STORAGE(0, 0, Color.kGreen), // intaked, coral or algae in
    REEF(0, -0.2, Color.kBlue), // scoring coral
    PROCESSOR(0, 0, Color.kPurple), // scoring algae
    CLIMB(0.3, 0, Color.kPink), // climbing
    EJECT(0.3, 0.2, Color.kWhite); // everything out

    public final double elevatorPosition;
    public final double coralIntakePosition;
    public final Color colorDisplay;

    private BotState(double elevatorSpeed, double coralIntakeSpeed, Color color) {
      this.elevatorPosition = elevatorSpeed;
      this.coralIntakePosition = coralIntakeSpeed;
      this.colorDisplay = color;
    }
  }
}
