// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  public final TalonFX climberMotor;
  public final Servo climberServo;

  private final Slot0Configs slot0Climber = new Slot0Configs();

  private final MotionMagicConfigs mmConfigs = new MotionMagicConfigs();

  private PositionVoltage positionVoltage = new PositionVoltage(0).withSlot(0);
  private DutyCycleOut dutyCycleOut;

  private final NeutralOut m_brake = new NeutralOut();

  /** Creates a new Climber. */
  public Climber() {
    climberMotor = new TalonFX(Constants.climberMotor);
    climberServo = new Servo(0);
    climberServo.setDisabled();

    slot0Climber.GravityType = GravityTypeValue.Arm_Cosine;
    slot0Climber.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;
    slot0Climber.kG = 0.0;
    slot0Climber.kS = 0.0;
    slot0Climber.kV = 0.0;
    slot0Climber.kA = 0.0;
    slot0Climber.kP = 1.0;
    slot0Climber.kI = 0.0;
    slot0Climber.kD = 0.0;


    mmConfigs.MotionMagicCruiseVelocity = 0;
    mmConfigs.MotionMagicAcceleration = 0;
    mmConfigs.MotionMagicJerk = 0;

    dutyCycleOut = new DutyCycleOut(0);

    // climberMotor.getConfigurator().apply(slot0Climber);
    // climberMotor.getConfigurator().apply(mmConfigs);

    /*
    if (Utils.isSimulation()) {
      PhysicsSim.getInstance().addTalonFX(climberMotor, 0.2);
    }
    */
    climberMotor.setPosition(0);
    climberClose();
  }

  public void moveClimberMotor(double position) {
    climberMotor.setControl(positionVoltage.withPosition(position));
  }

  public void climberMotorDutyCycle(double dutyCycle) {
    climberMotor.setControl(dutyCycleOut.withOutput(dutyCycle));
    // climberMotor.setControl(motionMagicVoltage.withPosition(velocity));
  }

  public void stopClimberMotor() {
    climberMotor.stopMotor();
  }

  public void climberOpen() {
    climberServo.setDisabled();
    climberServo.setAngle(180);
  }

  public void climberClose() {
    climberServo.setAngle(0);
    climberServo.setDisabled();
  }

  public void setBrake() {
    climberMotor.setControl(m_brake);
  }

  public Command cm_climberMovement() {
    return runEnd(() -> moveClimberMotor(5), () -> stopClimberMotor());
  }

  public Command cm_climberVelocity(double velocity) {
    return runEnd(() -> climberMotorDutyCycle(velocity), () -> stopClimberMotor());
  }

  public Command cm_solenoidToggle() {
    return Commands.sequence(
        runOnce(() -> climberOpen()).withTimeout(1), runOnce(() -> climberClose()).withTimeout(1));
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
