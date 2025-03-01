// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

public class CoralIntake extends SubsystemBase {
  final VoltageOut m_sysIdControl = new VoltageOut(0);

  public final TalonFX coralIntakeMotor;
  private final SysIdRoutine m_sysIdRoutine;

  private VelocityVoltage velocityVoltage = new VelocityVoltage(0).withSlot(0);

  /*private MotionMagicVelocityVoltage mmVelocityVoltage =
  new MotionMagicVelocityVoltage(0).withSlot(0); */

  /** Creates a new ExampleSubsystem. */
  public CoralIntake() {
    coralIntakeMotor = new TalonFX(8);

    m_sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null, // Use default ramp rate (1 V/s)
                Volts.of(2), // Reduce dynamic voltage to 4 to prevent brownout
                null, // Use default timeout (10 s)
                // Log state with Phoenix SignalLogger class
                state -> SignalLogger.writeString("Coral SYSID", state.toString())),
            new SysIdRoutine.Mechanism(
                volts -> coralIntakeMotor.setControl(m_sysIdControl.withOutput(volts)),
                null,
                this));
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }

  public void runIntakeMotor(double speed) {
    coralIntakeMotor.set(speed);
  }

  public void stopIntakeMotor() {
    coralIntakeMotor.stopMotor();
  }

  public void coralIntakeMotorVelocity(double velocity) {
    coralIntakeMotor.setControl(velocityVoltage.withVelocity(velocity));
  }

  public Command cm_intakeCoral(double speed) {
    return startEnd(() -> runIntakeMotor(speed), () -> stopIntakeMotor());
  }

  public Command cm_coralIntakeState() {
    return startEnd(
        () -> runIntakeMotor(Constants.StateMachineConstant.botState.coralIntakePosition),
        () -> stopIntakeMotor());
  }

  /* public Command cm_intakeCoralVelocity(double velocity) {
    return run(() -> coralIntakeMotorVelocity(velocity));
  }
  */
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
