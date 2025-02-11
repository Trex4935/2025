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
  public final TalonFX leftElevatorMotor, rightElevatorMotor;

  public Elevator() {
    leftElevatorMotor = new TalonFX(9);
    rightElevatorMotor = new TalonFX(10);
  }

  public void runLeftElevatorMotor(double speed) {
    leftElevatorMotor.set(speed);
  }

  public void runRightElevatorMotor(double speed) {
    rightElevatorMotor.set(speed);
  }

  public void runElevatorMotors(double leftSpeed, double rightSpeed) {
    runLeftElevatorMotor(leftSpeed);
    runRightElevatorMotor(rightSpeed);
  }

  public void stopLeftElevatorMotor() {
    leftElevatorMotor.stopMotor();
  }

  public void stopRightElevatorMotor() {
    rightElevatorMotor.stopMotor();
  }

  public void stopElevatorMotors() {
    stopLeftElevatorMotor();
    stopRightElevatorMotor();
  }

  // method to set the position of the elevator
  public void setElevatorPosition(double position) {
    leftElevatorMotor.setPosition(position);
  }

  public Command cm_elevatorMovement(double leftSpeed, double rightSpeed) {
    return startEnd(() -> runElevatorMotors(leftSpeed, rightSpeed), () -> stopElevatorMotors());
  }

  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty(
        "Left Elevator Encoder Position",
        () -> leftElevatorMotor.getPosition().getValueAsDouble(),
        null);
    builder.addDoubleProperty(
        "Right Elevator Encoder Position",
        () -> rightElevatorMotor.getPosition().getValueAsDouble(),
        null);
    builder.addDoubleProperty("Left Elevator percent output", () -> leftElevatorMotor.get(), null);
    builder.addDoubleProperty(
        "Right Elevator percent output", () -> rightElevatorMotor.get(), null);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
