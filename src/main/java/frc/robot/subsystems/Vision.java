// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.extensions.LimelightHelpers;

public class Vision extends SubsystemBase {
  /** Creates a new Vision. */
  public Vision() {}

  public double[] getPose() {
    double[] poseArray =
        new double[] {
          LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-bow").pose.getX(),
          LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-bow").pose.getY(),
          LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-bow")
              .pose
              .getRotation()
              .getRadians()
        };
    return poseArray;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    if (LimelightHelpers.getLatestResults("limelight-bow").error == null) {
    builder.addDoubleArrayProperty("Pose", () -> getPose(), null);
    builder.addDoubleProperty("tx", () -> LimelightHelpers.getTX("limelight-bow"), null);
    builder.addDoubleProperty("ty", () -> LimelightHelpers.getTY("limelight-bow"), null);
    builder.addDoubleProperty("ta", () -> LimelightHelpers.getTA("limelight-bow"), null);
    builder.addBooleanProperty("tv", () -> LimelightHelpers.getTV("limelight-bow"), null);
    }

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
