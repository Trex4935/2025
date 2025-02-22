// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.extensions.StateMachine;

public class CoralIntake extends SubsystemBase {
  public final TalonFX coralIntakeMotor;
  public final SparkMax coralPivotMotor;

  /** Creates a new ExampleSubsystem. */
  public CoralIntake() {
    coralIntakeMotor = new TalonFX(8);
    coralPivotMotor = new SparkMax(6, MotorType.kBrushless);
    SparkMaxConfig config = new SparkMaxConfig();

config.signals.primaryEncoderPositionPeriodMs(5);
config.inverted(true);
config.idleMode(IdleMode.kCoast);
coralPivotMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
double position = coralPivotMotor.getEncoder().getPosition();
  }

  public void runCoralPivotMotor(double speed) {
    coralPivotMotor.set(speed);
  }

  public void stopCoralPivotMotor() {
    coralPivotMotor.stopMotor();
  }


  // pivotPID.setReference(targetAngle, CANSparkBase.ControlType.kPosition);

  public void runIntakeMotor(double speed) {
    coralIntakeMotor.set(speed);
  }

  public void stopIntakeMotor() {
    coralIntakeMotor.stopMotor();
  }


  public Command cm_runCoralPivotMotor(double speed) {
    return startEnd(() -> runCoralPivotMotor(speed), () -> stopCoralPivotMotor());
  }


  public void coralIntakeMotorVelocity(double velocity) {
    coralIntakeMotor.setControl(new MotionMagicVelocityVoltage(velocity));
  }

  public Command cm_intakeCoral(double speed) {
    return startEnd(() -> runIntakeMotor(speed), () -> stopIntakeMotor());
  }

  public Command cm_coralIntakeState() {
    return startEnd(
        () -> runIntakeMotor(Constants.StateMachineConstant.botState.coralIntakeSpeed),
        () -> stopIntakeMotor());
  }

  public Command cm_intakeCoralVelocity(double velocity) {
    return startEnd(() -> coralIntakeMotorVelocity(velocity), () -> stopIntakeMotor());
  }

  public void setIntakePivotPosition(){
    //coralPivotMotor.getClosedLoopController().setReference(Constants.StateMachineConstant.botState.pivotAngle, ControlType.kPosition);
    coralPivotMotor.getClosedLoopController().setReference(0.1, ControlType.kPosition);
  }

  public Command cm_setIntakePivotPosition(){
    return startEnd(() -> setIntakePivotPosition(), null);
  }

  public void initSendable(SendableBuilder builder) {
    builder.addStringProperty(
        "botState", () -> Constants.StateMachineConstant.botState.toString(), null);
    builder.addDoubleProperty(
        "Coral intake motor percent output", () -> coralIntakeMotor.get(), null);
    builder.addDoubleProperty(
        "Coral intake motor velocity",
        () -> coralIntakeMotor.getVelocity().getValueAsDouble(),
        null);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
