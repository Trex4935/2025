// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  public final TalonFX climberMotor;

  private final Slot0Configs slot0Climber = new Slot0Configs();

  private final MotionMagicConfigs mmConfigs = new MotionMagicConfigs();

  private MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0).withSlot(0);

  /** Creates a new Climber. */
  public Climber() {
    climberMotor = new TalonFX(Constants.climberMotor);

    slot0Climber.GravityType = GravityTypeValue.Arm_Cosine;
    slot0Climber.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;
    slot0Climber.kG = 0.0;
    slot0Climber.kS = 0.0;
    slot0Climber.kV = 0.0;
    slot0Climber.kA = 0.0;
    slot0Climber.kP = 0.0;
    slot0Climber.kI = 0.0;
    slot0Climber.kD = 0.0;

    mmConfigs.MotionMagicCruiseVelocity = 0;
    mmConfigs.MotionMagicAcceleration = 0;
    mmConfigs.MotionMagicJerk = 0;

    climberMotor.getConfigurator().apply(slot0Climber);
    climberMotor.getConfigurator().apply(mmConfigs);
  }

  public void moveClimberMotor(double speed) {
    climberMotor.setControl(motionMagicVoltage.withPosition(speed));
  }

  public void climberMotorVelocity(double velocity) {
    climberMotor.setControl(motionMagicVoltage.withPosition(velocity));
  }

  public void stopClimberMotor() {
    climberMotor.stopMotor();
  }

  public Command climberMovement() {
    return runEnd(() -> moveClimberMotor(0.5), () -> stopClimberMotor());
  }

  public Command cm_climberVelocity(double velocity) {
    return startEnd(() -> climberMotorVelocity(velocity), () -> stopClimberMotor());
  }

  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty(
        "Climber Encoder Pos", () -> climberMotor.getPosition().getValueAsDouble(), null);
    builder.addDoubleProperty("Climber motor percent output", () -> climberMotor.get(), null);
    builder.addDoubleProperty(
        "Climber motor velocity", () -> climberMotor.getVelocity().getValueAsDouble(), null);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
