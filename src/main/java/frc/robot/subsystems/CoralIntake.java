// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.extensions.PhysicsSim;

public class CoralIntake extends SubsystemBase {
  final VoltageOut m_sysIdControl = new VoltageOut(0);

  public final TalonFX coralIntakeMotor;
  public final TalonFXS coralPivotMotor;
  private final SysIdRoutine m_sysIdRoutine;

  private final TalonFXSConfiguration coralPivotconfigs = new TalonFXSConfiguration();

  private VelocityVoltage velocityVoltage = new VelocityVoltage(0).withSlot(0);
  private MotionMagicVoltage mmVoltage = new MotionMagicVoltage(0).withSlot(0);

  private final NeutralOut m_brake = new NeutralOut();

  /*private MotionMagicVelocityVoltage mmVelocityVoltage =
  new MotionMagicVelocityVoltage(0).withSlot(0); */

  /** Creates a new ExampleSubsystem. */
  public CoralIntake() {
    coralIntakeMotor = new TalonFX(Constants.coralIntakeMotor);
    coralPivotMotor = new TalonFXS(Constants.coralPivotMotor);

    // TODO: Tune
    coralPivotconfigs.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    coralPivotconfigs.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;
    coralPivotconfigs.Slot0.kG = 0.0;
    coralPivotconfigs.Slot0.kS = 0.0;
    coralPivotconfigs.Slot0.kV = 1.0;
    coralPivotconfigs.Slot0.kP = 3.0;
    coralPivotconfigs.Slot0.kI = 0.0;
    coralPivotconfigs.Slot0.kD = 0.0;

    coralPivotconfigs.MotionMagic.MotionMagicCruiseVelocity = 0;
    coralPivotconfigs.MotionMagic.MotionMagicAcceleration = 0;
    coralPivotconfigs.MotionMagic.MotionMagicJerk = 0;

    // coralPivotMotor.getConfigurator().apply(slot0Pivot);
    // coralPivotMotor.getConfigurator().apply(mmConfigs);

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

    if (Utils.isSimulation()) {
      PhysicsSim.getInstance().addTalonFX(coralIntakeMotor, 0.2);
      PhysicsSim.getInstance().addTalonFXS(coralPivotMotor, 0.2);
    }
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }

  public void runCoralPivotMotor(double speed) {
    coralPivotMotor.set(speed);
  }

  public void stopCoralPivotMotor() {
    coralPivotMotor.stopMotor();
  }

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
    coralIntakeMotor.setControl(velocityVoltage.withVelocity(velocity));
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

  public void setIntakePivotPosition(double position) {
    coralPivotMotor.setControl(mmVoltage.withPosition(position));
  }

  public void setBrake() {
    coralPivotMotor.setControl(m_brake);
  }

  public Command cm_setIntakePivotPosition(double position) {
    return startEnd(() -> setIntakePivotPosition(position), () -> setBrake());
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
