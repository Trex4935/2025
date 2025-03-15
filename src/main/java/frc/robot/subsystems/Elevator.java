// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  public final TalonFX leftElevatorMotor, rightElevatorMotor;

  public final double maxElevatorRotation = 20;

  public final CANrange canRange;

  private final TalonFXConfiguration elevatorConfigs = new TalonFXConfiguration();

  private MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0).withSlot(0);

  private final NeutralOut m_brake = new NeutralOut();

  public Elevator() {
    leftElevatorMotor = new TalonFX(Constants.elevatorLeftID);
    rightElevatorMotor = new TalonFX(Constants.elevatorRightID);

    // TODO: Adjust feedback values
    elevatorConfigs.Slot0.GravityType = GravityTypeValue.Elevator_Static;
    elevatorConfigs.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;
    elevatorConfigs.Slot0.kG = 0.2;
    elevatorConfigs.Slot0.kV = 0.9;
    elevatorConfigs.Slot0.kA = 0.1;
    elevatorConfigs.Slot0.kP = 7.0;
    elevatorConfigs.Slot0.kI = 0.0;
    elevatorConfigs.Slot0.kD = 0.0;

    elevatorConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    elevatorConfigs.MotionMagic.MotionMagicCruiseVelocity = 100;
    elevatorConfigs.MotionMagic.MotionMagicAcceleration = 40;
    elevatorConfigs.MotionMagic.MotionMagicJerk = 0;

    elevatorConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    elevatorConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 70;
    elevatorConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    elevatorConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;

    leftElevatorMotor.getConfigurator().apply(elevatorConfigs);

    /* Make sure we start at 0 */
    leftElevatorMotor.setPosition(0);
    rightElevatorMotor.setPosition(0);

    rightElevatorMotor.setControl(new Follower(leftElevatorMotor.getDeviceID(), false));

    canRange = new CANrange(Constants.canRange);

    /*
    if (Utils.isSimulation()) {
      PhysicsSim.getInstance().addTalonFX(leftElevatorMotor, 0.2);
      PhysicsSim.getInstance().addTalonFX(rightElevatorMotor, 0.2);
    }
    */
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
