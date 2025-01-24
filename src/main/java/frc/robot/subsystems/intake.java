// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  public final TalonFX intakeMotor1;

  /** Creates a new ExampleSubsystem. */
  public Intake() {
    intakeMotor1 = new TalonFX(0);
  }

  public void pickUp() {
    intakeMotor1.set(.8);
  }

  public void stop() {
    intakeMotor1.stopMotor();
  }

  public void drop() {
    intakeMotor1.set(-.8);
  }

  public Command intakeGo() {
    return runEnd(() -> pickUp(), () -> stop());
  }

  public Command intakeDrop() {
    return runEnd(() -> drop(), () -> stop());
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


  public void initSendable(SendableBuilder builder){
    builder.addDoubleProperty("Intake motor 1 speed", () -> intakeMotor1.get(), null);
    builder.addDoubleProperty("Intake motor 1 velocity", () -> intakeMotor1.getVelocity().getValueAsDouble(), null);
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
