// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public class AlignmentLocations {

  private static AprilTagFieldLayout fieldAprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);

  public static class AlignmentPose {
    public Pose2d aprilTagPose;
    public Pose2d aprilTagPoseBlue;
    public Pose2d aprilTagPoseRed;
    public double[] offset;

    // Neutral tag pose values
    public double tagPoseX, tagPoseY, tagPoseThetaDegrees;
    public Rotation2d tagPoseTheta;

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
      this.aprilTagPoseRed = redTagPose;
      this.offset = offsetArray;

      this.blueTagPoseX = blueTagPose.getX();
      this.blueTagPoseY = blueTagPose.getY();
      this.blueTagPoseThetaDegrees = blueTagPose.getRotation().getDegrees();
      this.blueTagPoseTheta = blueTagPose.getRotation();

      this.redTagPoseX = redTagPose.getX();
      this.redTagPoseY = redTagPose.getY();
      this.redTagPoseThetaDegrees = redTagPose.getRotation().getDegrees();
      this.redTagPoseTheta = redTagPose.getRotation();

      this.offsetX = offsetArray[0];
      this.offsetY = offsetArray[1];
      this.offsetThetaDegrees = offsetArray[2];
      this.offsetTheta = Rotation2d.fromDegrees(offsetArray[2]);
    }

    AlignmentPose(Pose2d tagPose, double[] offsetArray) {
      this.aprilTagPose = tagPose;
      this.offset = offsetArray;

      this.tagPoseX = tagPose.getX();
      this.tagPoseY = tagPose.getY();
      this.tagPoseThetaDegrees = tagPose.getRotation().getDegrees();
      this.tagPoseTheta = tagPose.getRotation();

      this.offsetX = offsetArray[0];
      this.offsetY = offsetArray[1];
      this.offsetThetaDegrees = offsetArray[2];
      this.offsetTheta = Rotation2d.fromDegrees(offsetArray[2]);
    }
  }

  private static final double xReefDist = 0.25;

  // TODO: Adjust values as needed
  public static AlignmentPose coralStationLeft =
      new AlignmentPose(
          fieldAprilTagLayout.getTagPose(13).get().toPose2d(),
          fieldAprilTagLayout.getTagPose(1).get().toPose2d(),
          new double[] {xReefDist, xReefDist * Math.tan(Math.toRadians(126)), 126});
  public static AlignmentPose coralStationRight =
      new AlignmentPose(
          fieldAprilTagLayout.getTagPose(12).get().toPose2d(),
          fieldAprilTagLayout.getTagPose(2).get().toPose2d(),
          new double[] {xReefDist, xReefDist * Math.tan(Math.toRadians(-126)), -126});
  public static AlignmentPose processor =
      new AlignmentPose(
          fieldAprilTagLayout.getTagPose(3).get().toPose2d(),
          fieldAprilTagLayout.getTagPose(16).get().toPose2d(),
          new double[] {0, -0.35, 90});
  public static AlignmentPose bargeOpposingView =
      new AlignmentPose(
          fieldAprilTagLayout.getTagPose(4).get().toPose2d(),
          fieldAprilTagLayout.getTagPose(15).get().toPose2d(),
          new double[] {0, 0, 0});
  public static AlignmentPose bargeAllianceView =
      new AlignmentPose(
          fieldAprilTagLayout.getTagPose(14).get().toPose2d(),
          fieldAprilTagLayout.getTagPose(5).get().toPose2d(),
          new double[] {0, 0, 0});
  public static AlignmentPose reefCloseLeft =
      new AlignmentPose(
          fieldAprilTagLayout.getTagPose(19).get().toPose2d(),
          fieldAprilTagLayout.getTagPose(6).get().toPose2d(),
          new double[] {-xReefDist, -xReefDist * Math.tan(Math.toRadians(-60)), -60});
  public static AlignmentPose reefCloseMid =
      new AlignmentPose(
          fieldAprilTagLayout.getTagPose(18).get().toPose2d(),
          fieldAprilTagLayout.getTagPose(7).get().toPose2d(),
          new double[] {-xReefDist * 2, 0, 0});
  public static AlignmentPose reefCloseRight =
      new AlignmentPose(
          fieldAprilTagLayout.getTagPose(17).get().toPose2d(),
          fieldAprilTagLayout.getTagPose(8).get().toPose2d(),
          new double[] {-xReefDist, -xReefDist * Math.tan(Math.toRadians(60)), 60});
  public static AlignmentPose reefFarRight =
      new AlignmentPose(
          fieldAprilTagLayout.getTagPose(22).get().toPose2d(),
          fieldAprilTagLayout.getTagPose(9).get().toPose2d(),
          new double[] {xReefDist, xReefDist * Math.tan(Math.toRadians(120)), 120});
  public static AlignmentPose reefFarMid =
      new AlignmentPose(
          fieldAprilTagLayout.getTagPose(21).get().toPose2d(),
          fieldAprilTagLayout.getTagPose(10).get().toPose2d(),
          new double[] {xReefDist * 2, 0, -180});
  public static AlignmentPose reefFarLeft =
      new AlignmentPose(
          fieldAprilTagLayout.getTagPose(20).get().toPose2d(),
          fieldAprilTagLayout.getTagPose(11).get().toPose2d(),
          new double[] {xReefDist, xReefDist * Math.tan(Math.toRadians(-120)), -120});

  public static AlignmentPose[] reefTags = {
    reefCloseLeft, reefCloseMid, reefCloseRight, reefFarLeft, reefFarMid, reefFarRight
  };

  public static Pose2d[] reefPoseListRed = {
    reefCloseLeft.aprilTagPoseRed,
    reefCloseMid.aprilTagPoseRed,
    reefCloseRight.aprilTagPoseRed,
    reefFarLeft.aprilTagPoseRed,
    reefFarMid.aprilTagPoseRed,
    reefFarRight.aprilTagPoseRed
  };

  public static Pose2d[] reefPoseListBlue = {
    reefCloseLeft.aprilTagPoseBlue,
    reefCloseMid.aprilTagPoseBlue,
    reefCloseRight.aprilTagPoseBlue,
    reefFarLeft.aprilTagPoseBlue,
    reefFarMid.aprilTagPoseBlue,
    reefFarRight.aprilTagPoseBlue
  };
}
