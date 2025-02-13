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

  public CANrange canRange;

  public PIDController elevatorPID;
  private double pidCalc = 0;
  private double position = 0.37;
  private HashMap<String, Double> elevatorPosition;
  public static boolean atPosition = false;

  public Elevator() {
    elevatorPID = new PIDController(0.85, 0, 0); // ONLY SET THE P VALUE

    elevatorPID.setTolerance(0.1);

    leftElevatorMotor = new TalonFX(9);
    rightElevatorMotor = new TalonFX(10);

    leftElevatorMotor.setNeutralMode(NeutralModeValue.Brake);
    rightElevatorMotor.setNeutralMode(NeutralModeValue.Brake);

    canRange = new CANrange(2);

    elevatorPosition = new HashMap<>();
    elevatorPosition.put("Default", 0.37);
    elevatorPosition.put("L2", 0.520);
    elevatorPosition.put("L3", 0.705);
  }

  public void runElevatorMotors(double speed) {
    leftElevatorMotor.set(speed);
    rightElevatorMotor.set(speed);
  }

  public void setMotorToPIDCalc() {
    pidCalc = elevatorPID.calculate(canRange.getDistance().getValueAsDouble(), position);
    runElevatorMotors(pidCalc);
    /*
    if (MathUtil.isNear(pidCalc, leftElevatorMotor.getPosition().getValueAsDouble(), 0.1)){
      atPosition = true;
    }
    else {
      at
    } */
  }

  public boolean isAtPosition() {
    return elevatorPID.atSetpoint();
  }

  public void stopElevatorMotors() {
    leftElevatorMotor.stopMotor();
    rightElevatorMotor.stopMotor();
  }

  public void setElevatorState(String targetPosition) {
    position = elevatorPosition.get(targetPosition);
  }

  public Command cm_setElevatorState(String targetState) {
    return runOnce(() -> setElevatorState(targetState));
  }

  // method to set the position of the elevator

  public Command cm_elevatorMovement(double speed) {
    return startEnd(() -> runElevatorMotors(speed), () -> stopElevatorMotors());
  }

  public double returnPID() {
    return pidCalc;
  }

  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty(
        "Left Elevator Encoder Position",
        () -> leftElevatorMotor.getPosition().getValueAsDouble(),
        null);
    builder.addDoubleProperty(
        "Right Elevator Encoder Position",
        () -> rightElevatorMotor.getPosition().getValueAsDouble(),
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

    builder.addDoubleProperty("PID Value", () -> returnPID(), null);
  }

  @Override
  public void periodic() {}
}
