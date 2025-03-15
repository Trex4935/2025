// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.extensions.PhysicsSim;

public class Climber extends SubsystemBase {
  final VoltageOut m_sysIdControlCilmb = new VoltageOut(0);
  private final SysIdRoutine m_sysIdCilmb;

  public final TalonFX climberMotor;

  private final Slot0Configs slot0Climber = new Slot0Configs();

  private final MotionMagicConfigs mmConfigs = new MotionMagicConfigs();

  private MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0).withSlot(0);
  private DutyCycleOut dutyCycleOut;
  private final NeutralOut m_brake = new NeutralOut();


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

    dutyCycleOut = new DutyCycleOut(0);

    // climberMotor.getConfigurator().apply(slot0Climber);
    // climberMotor.getConfigurator().apply(mmConfigs);

    /*
    if (Utils.isSimulation()) {
      PhysicsSim.getInstance().addTalonFX(climberMotor, 0.2);
    }
  }

  public void moveClimberMotor(double position) {
    climberMotor.setControl(motionMagicVoltage.withPosition(position));
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

  public void climberOpen() {
    climberSolenoid.set(true);
  }

  public void setBrake(){
    climberMotor.setControl(m_brake);
  }

  public boolean getClimberState() {
    return climberSolenoid.get();
  }

  public Command cm_climberMovement(double position) {
    return runEnd(() -> moveClimberMotor(position), () -> stopClimberMotor());
  }

  public Command cm_climberVelocity(double velocity) {
    return runEnd(() -> climberMotorDutyCycle(velocity), () -> stopClimberMotor());
  }

  public Command cm_solenoidToggle() {
    return runOnce(() -> climberOpen()).withTimeout(1).andThen(runOnce(() -> climberClose()));
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
