// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  public final TalonFX elevatorMotor;

  public Elevator() {
    elevatorMotor = new TalonFX(9);
  }

  public void elevatorMoveUp(double speed) {
    elevatorMotor.set(speed);
  }

  public void elevatorMoveDown(double speed) {
    elevatorMotor.set(-speed);
  }

  public void stopElevator() {
    elevatorMotor.stopMotor();
  }

  public Command cm_movementUp() {
    return startEnd(() -> elevatorMoveUp(0.3), () -> stopElevator());
  }

  public Command cm_movementDown() {
    return startEnd(() -> elevatorMoveDown(0.3), () -> stopElevator());
  }

  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty(
        "Left Climber Encoder Position", () -> elevatorMotor.getPosition().getValueAsDouble(), null);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
