// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  public final TalonFX climberMotor;
  private MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0).withSlot(1);

  /** Creates a new Climber. */
  public Climber() {
    climberMotor = new TalonFX(7);
  }

  public void moveClimberMotor(double speed) {
    climberMotor.setControl(motionMagicVoltage.withPosition(speed));
  }

  public void stopClimberMotor() {
    climberMotor.stopMotor();
  }

  public Command climberMovement() {
    return runEnd(() -> moveClimberMotor(0.5), () -> stopClimberMotor());
  }

  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty(
        "Climber Encoder Pos", () -> climberMotor.getPosition().getValueAsDouble(), null);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
