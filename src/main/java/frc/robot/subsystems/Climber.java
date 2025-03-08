// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  public final TalonFX climberMotor;
  private MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0).withSlot(1);
  Solenoid climberSolenoid;

  /** Creates a new Climber. */
  public Climber() {
    climberMotor = new TalonFX(7);
    climberSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
  }

  public void moveClimberMotor(double speed) {
    climberMotor.setControl(motionMagicVoltage.withPosition(speed));
  }

  public void climberMotorVelocity(double velocity) {
    climberMotor.setControl(motionMagicVoltage.withPosition(velocity));
  }

  public void climberOpen() {
    climberSolenoid.set(true);
  }

  public void stopClimberMotor() {
    climberMotor.stopMotor();
  }

  public void climberClose() {
    climberSolenoid.set(false);
  }

  public Command climberMovement() {
    return runEnd(() -> moveClimberMotor(0.5), () -> stopClimberMotor());
  }

  public Command cm_climberVelocity(double velocity) {
    return startEnd(() -> climberMotorVelocity(velocity), () -> stopClimberMotor());
  }

  public boolean getClimberState() {
    return climberSolenoid.get();
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
