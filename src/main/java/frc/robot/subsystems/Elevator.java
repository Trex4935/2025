// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.extensions.StateMachine.RobotStateMachine;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  public final TalonFX leftElevatorMotor, rightElevatorMotor;

  public CANrange canRange;

  public PIDController elevatorPID;
  private double pidCalc = 0;
  private double position = 0.37;
  public static boolean atPosition = false;

  public Elevator() {
    leftElevatorMotor = new TalonFX(9);
    rightElevatorMotor = new TalonFX(10);

    leftElevatorMotor.setNeutralMode(NeutralModeValue.Brake);
    rightElevatorMotor.setNeutralMode(NeutralModeValue.Brake);

    elevatorPID = new PIDController(0.85, 0, 0); // ONLY SET THE P VALUE

    canRange = new CANrange(2);
  }

  public void runElevatorMotors(double speed) {
    leftElevatorMotor.set(speed);
    rightElevatorMotor.set(speed);
  }

  public void setMotorToPIDCalc() {
    System.out.println(RobotStateMachine.botState);
    pidCalc = elevatorPID.calculate(canRange.getDistance().getValueAsDouble(), position);
    runElevatorMotors(pidCalc);

    if (MathUtil.isNear(position, canRange.getDistance().getValueAsDouble(), 0.1)) {
      atPosition = true;
    } else {
      atPosition = false;
    }
  }

  public boolean isAtPosition() {
    return elevatorPID.atSetpoint();
  }

  public void stopElevatorMotors() {
    leftElevatorMotor.stopMotor();
    rightElevatorMotor.stopMotor();
  }

  public void setElevatorPosition(Double targetPosition) {
    // code to move elevator position to targetPosition goes here
    System.out.println(RobotStateMachine.botState);
    position = targetPosition;
  }

  public Command cm_setElevatorPosition(Double targetPosition) {
    return runOnce(() -> setElevatorPosition(targetPosition));
  }

  // method to set the position of the elevator

  public Command cm_elevatorMovement(double speed) {
    return startEnd(() -> runElevatorMotors(speed), () -> stopElevatorMotors());
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
  public void periodic() {}
}
