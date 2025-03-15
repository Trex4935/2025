// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
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
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  final VoltageOut m_sysIdControlCilmb = new VoltageOut(0);
  private final SysIdRoutine m_sysIdCilmb;

  public final TalonFX climberMotor;

  private final Slot0Configs slot0Climber = new Slot0Configs();

  private final MotionMagicConfigs mmConfigs = new MotionMagicConfigs();

  public static PowerDistribution powerDistributionSwitch =
      new PowerDistribution(1, ModuleType.kRev);

  private MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0).withSlot(0);
  private DutyCycleOut dutyCycleOut;
  private final NeutralOut m_brake = new NeutralOut();

  /** Creates a new Climber. */
  public Climber() {
    powerDistributionSwitch.setSwitchableChannel(false);

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
    */
    m_sysIdCilmb =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null, // Use default ramp rate (1 V/s)
                Volts.of(1), // Reduce dynamic voltage to 4 to prevent brownout
                null, // Use default timeout (10 s)
                // Log state with Phoenix SignalLogger class
                state -> SignalLogger.writeString("something SYSID", state.toString())),
            new SysIdRoutine.Mechanism(
                volts -> climberMotor.setControl(m_sysIdControlCilmb.withOutput(volts)),
                null,
                this));
  }

  public Command sysIdQuasistatiClim(SysIdRoutine.Direction direction) {
    return m_sysIdCilmb.quasistatic(direction);
  }

  public Command sysIdDynamicCilm(SysIdRoutine.Direction direction) {
    return m_sysIdCilmb.dynamic(direction);
  }

  public void moveClimberMotor(double position) {
    climberMotor.setControl(motionMagicVoltage.withPosition(position));
  }

  public void climberMotorDutyCycle(double dutyCycle) {
    climberMotor.setControl(dutyCycleOut.withOutput(dutyCycle));
    // climberMotor.setControl(motionMagicVoltage.withPosition(velocity));
  }

  public void stopClimberMotor() {
    climberMotor.stopMotor();
  }

  public void climberOpen() {
    powerDistributionSwitch.setSwitchableChannel(true);
  }

  public void climberClose() {
    powerDistributionSwitch.setSwitchableChannel(false);
  }

  public void setBrake() {
    climberMotor.setControl(m_brake);
  }

  public boolean getClimberState() {
    return powerDistributionSwitch.getSwitchableChannel();
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

  public void initSendable(SendableBuilder builder) {
    SmartDashboard.setDefaultBoolean("Set Switchable Channel", false);

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
