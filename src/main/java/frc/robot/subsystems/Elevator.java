// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.util.sendable.SendableBuilder;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  public final TalonFX moveElevator;

  public Elevator() {
    moveElevator = new TalonFX(9);
  }

  public void elevatorMoveUp() {
    moveElevator.set(0.3);
  }

  public void elevatorMoveDown() {
    moveElevator.set(-0.3);
  }

  public void stopElevator() {
    moveElevator.stopMotor();
  }




  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty(
        "Left Climber Encoder Position", () -> moveElevator.getPosition().getValueAsDouble(), null);
  }


  public Command cm_movementUp() {
    return startEnd(() -> elevatorMoveUp(), () -> stopElevator());
  }

  public Command cm_movementDown() {
    return startEnd(() -> elevatorMoveDown(), () -> stopElevator());
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
