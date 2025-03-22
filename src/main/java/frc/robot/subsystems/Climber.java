// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  public final TalonFX climberMotor;

  private final TalonFXConfiguration climberConfigs = new TalonFXConfiguration();

  private PositionVoltage positionVoltage = new PositionVoltage(0).withSlot(0);
  private DutyCycleOut dutyCycleOut;

  private final NeutralOut m_brake = new NeutralOut();

  private Solenoid solenoid;

  /** Creates a new Climber. */
  public Climber() {
    climberMotor = new TalonFX(Constants.climberMotor);
    solenoid = new Solenoid(14, PneumaticsModuleType.CTREPCM, 5);

    climberConfigs.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    climberConfigs.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;
    climberConfigs.Slot0.kG = 0.0;
    climberConfigs.Slot0.kS = 0.0;
    climberConfigs.Slot0.kV = 0.0;
    climberConfigs.Slot0.kA = 0.0;
    climberConfigs.Slot0.kP = 1.0;
    climberConfigs.Slot0.kI = 0.0;
    climberConfigs.Slot0.kD = 0.0;

    dutyCycleOut = new DutyCycleOut(0);

    climberMotor.getConfigurator().apply(climberConfigs);

    /*
    if (Utils.isSimulation()) {
      PhysicsSim.getInstance().addTalonFX(climberMotor, 0.2);
    }
    */
    climberMotor.setPosition(0);
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
    solenoid.set(true);
  }

  public void climberClose() {
    solenoid.set(false);
  }

  public void setBrake() {
    climberMotor.setControl(m_brake);
  }

  public Command cm_open() {
    return run(() -> climberOpen());
  }

  public Command cm_close() {
    return run(() -> climberClose());
  }

  public Command cm_climberMovement() {
    return runEnd(() -> moveClimberMotor(5), () -> stopClimberMotor());
  }

  public Command cm_climberVelocity(double velocity) {
    return runEnd(() -> climberMotorDutyCycle(velocity), () -> stopClimberMotor());
  }

  public Command cm_solenoidToggle() {
    return Commands.sequence(cm_open(), new WaitCommand(1), cm_close());
  }

  @Override
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
