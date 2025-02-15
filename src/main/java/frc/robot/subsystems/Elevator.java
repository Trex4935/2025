// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import static edu.wpi.first.units.Units.Volts;

import java.util.HashMap;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  public final TalonFX leftElevatorMotor, rightElevatorMotor;

  public CANrange canRange;

  public PIDController elevatorPID;
  private double pidCalc = 0;
  private double position = 0.37;
  private HashMap<String, Double> elevatorPosition;
  public static boolean atPosition = false;
  SysIdRoutine m_sysIdRoutine;



  public Elevator() {
    elevatorPID = new PIDController(0.85, 0, 0); // ONLY SET THE P VALUE

    elevatorPID.setTolerance(0.1);

    leftElevatorMotor = new TalonFX(9);
    rightElevatorMotor = new TalonFX(10);
    leftElevatorMotor.setControl(new StrictFollower(rightElevatorMotor.getDeviceID()));

    leftElevatorMotor.setNeutralMode(NeutralModeValue.Brake);
    rightElevatorMotor.setNeutralMode(NeutralModeValue.Brake);
    final VoltageOut m_sysIdControl = new VoltageOut(0);

    canRange = new CANrange(2);

    elevatorPosition = new HashMap<>();
    elevatorPosition.put("Default", 0.37);
    elevatorPosition.put("L2", 0.520);
    elevatorPosition.put("L3", 0.705);

    final SysIdRoutine m_sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,         // Use default ramp rate (1 V/s)
                Volts.of(4), // Reduce dynamic voltage to 4 to prevent brownout
                null,          // Use default timeout (10 s)
                                       // Log state with Phoenix SignalLogger class
                state -> SignalLogger.writeString("Elevator SYSID", state.toString())
            ),
            new SysIdRoutine.Mechanism(
                volts -> rightElevatorMotor.setControl(m_sysIdControl.withOutput(volts)),
                null,
                this
            )
        );
  }

  public void runElevatorMotors(double speed) {
    leftElevatorMotor.set(speed);
    rightElevatorMotor.set(speed);
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
}
public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
}

  public void setMotorToPIDCalc() {
    pidCalc = elevatorPID.calculate(canRange.getDistance().getValueAsDouble(), position);
    runElevatorMotors(pidCalc);
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

    builder.addDoubleProperty("PID Value", () -> returnPID(), null);
  }

  @Override
  public void periodic() {}
}
