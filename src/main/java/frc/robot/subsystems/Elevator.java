// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  public final TalonFX leftElevatorMotor, rightElevatorMotor;

  final VoltageOut m_sysIdControl = new VoltageOut(0);

  public final double maxElevatorRotation = 20;

  public final CANrange canRange;
  private final SysIdRoutine m_sysIdEle;

  private MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0).withSlot(0);

  private final NeutralOut m_brake = new NeutralOut();

  public Elevator() {

    leftElevatorMotor = new TalonFX(9);
    rightElevatorMotor = new TalonFX(10);

    /* Make sure we start at 0 */
    leftElevatorMotor.setPosition(0);
    rightElevatorMotor.setPosition(0);

    rightElevatorMotor.setControl(new Follower(leftElevatorMotor.getDeviceID(), false));

    canRange = new CANrange(2);
    m_sysIdEle =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null, // Use default ramp rate (1 V/s)
                Volts.of(2), // Reduce dynamic voltage to 4 to prevent brownout
                null, // Use default timeout (10 s)
                // Log state with Phoenix SignalLogger class
                state -> SignalLogger.writeString("Ele SYSID", state.toString())),
            new SysIdRoutine.Mechanism(
                volts -> leftElevatorMotor.setControl(m_sysIdControl.withOutput(volts)),
                null,
                this));
  }

  public void setElevatorPosition(double position) {
    leftElevatorMotor.setControl(motionMagicVoltage.withPosition(position));
  }

  public void setBrake() {
    leftElevatorMotor.setControl(m_brake);
  }

  public void stopElevator() {
    leftElevatorMotor.stopMotor();
  }

  public void moveElevator(double speed) {
    leftElevatorMotor.set(speed);
  }

  public Command cm_setElevatorPosition(double position) {
    return run(() -> setElevatorPosition(position));
  }

  /*public Command cm_setElevatorPositionRunOnce(double position) {
    return runOnce(() -> setElevatorPosition(position));
  }
  */
  public final Command cm_setElevatorToState() {
    return run(
        () -> setElevatorPosition(Constants.StateMachineConstant.getState().elevatorPosition));
  }

  public final Command cm_moveElevator(double speed) {
    return startEnd(() -> moveElevator(speed), () -> stopElevator());
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
