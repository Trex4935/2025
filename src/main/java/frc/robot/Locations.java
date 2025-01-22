// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public class Locations {

  private static AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

  public static class AlignmentPose {
    public Pose2d aprilTagPoseBlue;
    public Pose2d aprilTagPoseRed;
    public double[] offset;

    // Blue tag pose values
    public double blueTagPoseX, blueTagPoseY, blueTagPoseThetaDegrees;
    public Rotation2d blueTagPoseTheta;

    // Red tag pose values
    public double redTagPoseX, redTagPoseY, redTagPoseThetaDegrees;
    public Rotation2d redTagPoseTheta;

    // Tag offset values
    public double offsetX, offsetY, offsetThetaDegrees;
    public Rotation2d offsetTheta;

    AlignmentPose(Pose2d blueTagPose, Pose2d redTagPose, double[] offsetArray) {
      this.aprilTagPoseBlue = blueTagPose;
      this.offset = offsetArray;

      this.aprilTagPoseRed = redTagPose;

      this.redTagPoseX = blueTagPose.getX();
      this.redTagPoseY = blueTagPose.getY();
      this.redTagPoseThetaDegrees = blueTagPose.getRotation().getDegrees();
      this.redTagPoseTheta = blueTagPose.getRotation();

      this.offsetX = offsetArray[0];
      this.offsetY = offsetArray[1];
      this.offsetThetaDegrees = offsetArray[2];
      this.offsetTheta = Rotation2d.fromDegrees(offsetArray[2]);

      this.redTagPoseY = redTagPose.getY();
      this.redTagPoseThetaDegrees = redTagPose.getRotation().getDegrees();
      this.redTagPoseTheta = redTagPose.getRotation();
      this.redTagPoseX = redTagPose.getX();
    }
  }

  // TODO: Adjust values as needed
  public static AlignmentPose coralStationLeft =
      new AlignmentPose(
          aprilTagLayout.getTagPose(13).get().toPose2d(),
          aprilTagLayout.getTagPose(1).get().toPose2d(),
          new double[] {0.35, 0.35 * Math.tan(Math.toRadians(126)), 126});
  public static AlignmentPose coralStationRight =
      new AlignmentPose(
          aprilTagLayout.getTagPose(12).get().toPose2d(),
          aprilTagLayout.getTagPose(2).get().toPose2d(),
          new double[] {0.35, 0.35 * Math.tan(Math.toRadians(-126)), -126});
  public static AlignmentPose processor =
      new AlignmentPose(
          aprilTagLayout.getTagPose(3).get().toPose2d(),
          aprilTagLayout.getTagPose(16).get().toPose2d(),
          new double[] {0, -0.7, 90});
  public static AlignmentPose bargeOpposingView =
      new AlignmentPose(
          aprilTagLayout.getTagPose(4).get().toPose2d(),
          aprilTagLayout.getTagPose(15).get().toPose2d(),
          new double[] {0, 0, 0});
  public static AlignmentPose bargeAllianceView =
      new AlignmentPose(
          aprilTagLayout.getTagPose(14).get().toPose2d(),
          aprilTagLayout.getTagPose(5).get().toPose2d(),
          new double[] {0, 0, 0});
  public static AlignmentPose reefCloseLeft =
      new AlignmentPose(
          aprilTagLayout.getTagPose(19).get().toPose2d(),
          aprilTagLayout.getTagPose(6).get().toPose2d(),
          new double[] {-0.35, -0.35 * Math.tan(Math.toRadians(-60)), -60});
  public static AlignmentPose reefCloseMid =
      new AlignmentPose(
          aprilTagLayout.getTagPose(18).get().toPose2d(),
          aprilTagLayout.getTagPose(7).get().toPose2d(),
          new double[] {-0.7, 0, 0});
  public static AlignmentPose reefCloseRight =
      new AlignmentPose(
          aprilTagLayout.getTagPose(17).get().toPose2d(),
          aprilTagLayout.getTagPose(8).get().toPose2d(),
          new double[] {-0.35, -0.35 * Math.tan(Math.toRadians(60)), 60});
  public static AlignmentPose reefFarRight =
      new AlignmentPose(
          aprilTagLayout.getTagPose(22).get().toPose2d(),
          aprilTagLayout.getTagPose(9).get().toPose2d(),
          new double[] {0.35, 0.35 * Math.tan(Math.toRadians(120)), 120});
  public static AlignmentPose reefFarMid =
      new AlignmentPose(
          aprilTagLayout.getTagPose(21).get().toPose2d(),
          aprilTagLayout.getTagPose(10).get().toPose2d(),
          new double[] {0.7, 0, -180});
  public static AlignmentPose reefFarLeft =
      new AlignmentPose(
          aprilTagLayout.getTagPose(20).get().toPose2d(),
          aprilTagLayout.getTagPose(11).get().toPose2d(),
          new double[] {0.35, 0.35 * Math.tan(Math.toRadians(-120)), -120});
}
