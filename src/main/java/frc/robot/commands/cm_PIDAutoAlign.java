// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class cm_PIDAutoAlign extends Command {
  private final CommandSwerveDrivetrain m_drivetrain;
  private final SwerveRequest.FieldCentric driveRequest;
  private PhoenixPIDController swervePIDx, swervePIDy, swervePIDtheta;
  private final Pose2d targetPose;

  /**
   * Aligns to a pose using a PID and feedforward values.
   *
   * @param targetPose
   * @param drivetrain
   */
  public cm_PIDAutoAlign(Pose2d targetPose, CommandSwerveDrivetrain drivetrain) {
    m_drivetrain = drivetrain;
    this.targetPose = targetPose;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);

    driveRequest = new SwerveRequest.FieldCentric();

    swervePIDx = new PhoenixPIDController(0.5, 0, 0);
    swervePIDy = new PhoenixPIDController(0.5, 0, 0);
    swervePIDtheta = new PhoenixPIDController(0.3, 0, 0);

    swervePIDtheta.enableContinuousInput(-180, 180);

    swervePIDx.setTolerance(0.05);
    swervePIDy.setTolerance(0.05);
    swervePIDtheta.setTolerance(Math.toRadians(0.1));
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    swervePIDx.reset();
    swervePIDy.reset();
    swervePIDtheta.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d pose = m_drivetrain.getState().Pose;
    Translation2d position = pose.getTranslation();
    Rotation2d rotation = pose.getRotation();

    double xFB =
        swervePIDx.calculate(position.getX(), targetPose.getX(), m_drivetrain.getState().Timestamp);
    double xFF = 0.02 * Math.signum(xFB);
    double xOut = MathUtil.clamp(xFB + xFF, -2.5, 2.5);

    double yFB =
        swervePIDy.calculate(position.getY(), targetPose.getY(), m_drivetrain.getState().Timestamp);
    double yFF = 0.02 * Math.signum(yFB);
    double yOut = MathUtil.clamp(yFB + yFF, -2.5, 2.5);

    double rotFB =
        swervePIDtheta.calculate(
            rotation.getDegrees(),
            targetPose.getRotation().getDegrees(),
            m_drivetrain.getState().Timestamp);
    double rotFF = 0.02 * Math.signum(rotFB);
    double rotOut = MathUtil.clamp(rotFB + rotFF, -2.5, 2.5);

    m_drivetrain.setControl(
        driveRequest
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.Position)
            .withVelocityX(xOut)
            .withVelocityY(yOut)
            .withRotationalRate(rotOut));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return swervePIDx.atSetpoint() && swervePIDy.atSetpoint() && swervePIDtheta.atSetpoint();
  }
}
