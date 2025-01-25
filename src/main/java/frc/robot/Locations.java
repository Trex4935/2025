// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import java.util.List;

/** Add your docs here. */
public class Locations {

  private static AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

  private static Pose3d at1p3d = new Pose3d(3.073, 1.6096, 1.1684, new Rotation3d(0, 0, Math.PI));
  private static Pose3d at2p3d = new Pose3d(0, -1.5875, 1.2549, new Rotation3d());
  private static Pose3d at3p3d = new Pose3d(0, 1.5875, 1.2319, new Rotation3d());
  private static Pose3d at4p3d = new Pose3d(0, 0, 1.2319, new Rotation3d());

  public static AprilTag at1 = new AprilTag(1, at1p3d);
  private static AprilTag at2 = new AprilTag(2, at2p3d);
  private static AprilTag at3 = new AprilTag(3, at3p3d);
  private static AprilTag at4 = new AprilTag(4, at4p3d);

  private static List<AprilTag> hallwayFieldMapTagList = List.of(at1, at2, at3, at4);

  private static AprilTagFieldLayout testAprilTagFieldLayout =
      new AprilTagFieldLayout(hallwayFieldMapTagList, 17.5483, 8.0519);

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

  public static AlignmentPose halltag1 =
      new AlignmentPose(
          testAprilTagFieldLayout.getTagPose(1).get().toPose2d(), new double[] {-1, 0, 0});
}
