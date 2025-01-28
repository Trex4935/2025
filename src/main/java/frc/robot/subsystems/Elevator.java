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

  public void runElevatorMotor(double speed) {
    elevatorMotor.set(speed);
  }

  public void stopElevatorMotor() {
    elevatorMotor.stopMotor();
  }

  public Command cm_elevatorMovement(double speed) {
    return startEnd(() -> runElevatorMotor(speed), () -> stopElevatorMotor());
  }

  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty(
        "Left Climber Encoder Position",
        () -> elevatorMotor.getPosition().getValueAsDouble(),
        null);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
