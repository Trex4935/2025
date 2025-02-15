// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.extensions;

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
    DEFAULT(0.37, 0, "Red"), // field state, default state, no game pieces
    INTAKECORAL(0.520, 0.2, "Orange"), // preparing to intake coral
    INTAKEALGAE(0.37, 0, "Yellow"), // preparing to intake algae
    STORAGE(0.37, 0, "Green"), // intaked, coral or algae in
    L1(0.37, -0.2, "Blue"), // scoring coral
    L2(0.520, -0.2, "Blue"), // scoring coral
    L3(0.705, -0.2, "Blue"), // scoring coral
    PROCESSOR(0.37, 0, "Purple"), // scoring algae
    CLIMB(0.37, 0, "Pink"), // climbing
    EJECT(0.37, -0.2, "White"); // everything out

    public final double elevatorPosition;
    public final double coralIntakeSpeed;
    public final String colorDisplay;

    private BotState(double elevatorPosition, double coralIntakeSpeed, String color) {
      this.elevatorPosition = elevatorPosition;
      this.coralIntakeSpeed = coralIntakeSpeed;
      this.colorDisplay = color;
    }
  }

  public static class StateMachineLogic {
    public static BotState botState;
    public static boolean atPosition = false;

    StateMachineLogic() {
      botState = BotState.DEFAULT;
    }

    public static BotState switchState(BotState state) {
      botState = state;
      return state;
    }
  }
}
