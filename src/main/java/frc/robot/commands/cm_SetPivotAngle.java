// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.StateMachineConstant;
import frc.robot.subsystems.CoralIntake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class cm_SetPivotAngle extends Command {
  private final CoralIntake m_coralIntake;

  /** Creates a new pivotAngle. */
  public cm_SetPivotAngle(CoralIntake coralIntake) {

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
    m_coralIntake.setIntakePivotPosition(StateMachineConstant.botState.pivotAngle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_coralIntake.run(() -> m_coralIntake.setBrake());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return MathUtil.isNear(
        StateMachineConstant.botState.pivotAngle,
        m_coralIntake.coralPivotMotor.getPosition().getValueAsDouble(),
        0.1);
  }
}
