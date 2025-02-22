// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralIntake;

/** An example command that uses an example subsystem. */
public class cm_SetCoralIntakeVelocity extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final CoralIntake m_coralIntake;

  Timer timer = new Timer();

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public cm_SetCoralIntakeVelocity(CoralIntake coralIntake) {
    m_coralIntake = coralIntake;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(coralIntake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_coralIntake.coralIntakeMotorVelocity(-5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_coralIntake.run(() -> m_coralIntake.setBrake());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(5);
  }
}
