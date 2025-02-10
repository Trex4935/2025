// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;

import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.CANrangeSimState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  public final TalonFX leftElevatorMotor, rightElevatorMotor;
  public CANrange canRange;
  public PIDController PID;
  private double pidCalc = 0;
  private double position = 0.37;
  private HashMap<String, Double> elevatorPosition;

  public Elevator() {
    PID = new PIDController(0.7, 0, 0);//ONLY SET THE P VALUE


    leftElevatorMotor = new TalonFX(9);
    rightElevatorMotor = new TalonFX(10);

    leftElevatorMotor.setNeutralMode(NeutralModeValue.Brake);
    rightElevatorMotor.setNeutralMode(NeutralModeValue.Brake);

    canRange = new CANrange(2);

    elevatorPosition = new HashMap<>();
    elevatorPosition.put("Default", 0.37);
    elevatorPosition.put("L2", 0.420);
    elevatorPosition.put("L3", 0.67);
  }


  public void runElevatorMotors(double speed) {
    leftElevatorMotor.set(speed);
    rightElevatorMotor.set(speed);
  }

  public void setMotorToPIDCalc() {
    pidCalc = PID.calculate(canRange.getDistance().getValueAsDouble(), position);
    runElevatorMotors(pidCalc);
  }

  public void stopElevatorMotors() {
    leftElevatorMotor.stopMotor();
    rightElevatorMotor.stopMotor();
  }

  public void setElevatorState(String targetPosition){
    position = elevatorPosition.get(targetPosition);
  }

  public Command cm_setElevatorState(String targetState){
    return runOnce(() -> setElevatorState(targetState));
  }

  // method to set the position of the elevator

  public Command cm_elevatorMovement(double speed) {
    return startEnd(() -> runElevatorMotors(speed), () -> stopElevatorMotors());
  }

  public double returnPID(){
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
        null);
    builder.addDoubleProperty("Left Elevator percent output", () -> leftElevatorMotor.get(), null);
    builder.addDoubleProperty("Right Elevator percent output", () -> rightElevatorMotor.get(), null);

    builder.addDoubleProperty("PID Value", () -> returnPID(), null);
  }

  @Override
  public void periodic() {
  }
}
