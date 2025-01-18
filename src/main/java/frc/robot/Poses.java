// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public class Poses {

    public static AprilTagFieldLayout aprilTagLayout =
    // TODO: Change to Reefscape when released
    AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);


    // All AprilTag poses
    public static List<Pose2d> aprilTagPoses = new ArrayList<Pose2d>(22);
    // Poses used by your alliance (extra tags not included)
    public static List<Pose2d> allianceAprilTags = new ArrayList<Pose2d>(11);
    // Poses used by the opposing alliance
    public static List<Pose2d> opposingAllianceAprilTags = new ArrayList<Pose2d>(11);
    // Pose provided by speaker
    public static Pose2d speakerAprilTag;
    // Poses provided by stage
    public static List<Pose2d> stageAprilTags = new ArrayList<>(3);
    // Pose provided by human player station (source)
    public static Pose2d sourceAprilTag;
    // Pose provided by amp
    public static Pose2d ampAprilTag;



    public static class AlignmentPose {
        public Pose2d aprilTagPoseBlue;
        public Pose2d aprilTagPoseRed;
        public double[] offsetBlue;
        public double[] offsetRed;

        // Blue tag pose values
        public double blueTagPoseX, blueTagPoseY, blueTagPoseThetaDegrees;
        public Rotation2d blueTagPoseTheta;

        // Blue tag offset values
        public double blueOffsetX, blueOffsetY, blueOffsetThetaDegrees;
        public Rotation2d blueOffsetTheta;

        // Red tag pose values
        public double redTagPoseX, redTagPoseY, redTagPoseThetaDegrees;
        public Rotation2d redTagPoseTheta;

        // Red tag offset values
        public double redOffsetX, redOffsetY, redOffsetThetaDegrees;
        public Rotation2d redOffsetTheta;

        AlignmentPose(Pose2d blueTagPose, double[] blueOffsetArray, Pose2d redTagPose, double[] redOffsetArray) {
            this.aprilTagPoseBlue = blueTagPose;
            this.offsetBlue = blueOffsetArray;

            this.aprilTagPoseRed = redTagPose;
            this.offsetRed = redOffsetArray;

            this.redTagPoseX = blueTagPose.getX();
            this.redTagPoseY = blueTagPose.getY();
            this.redTagPoseThetaDegrees = blueTagPose.getRotation().getDegrees();
            this.redTagPoseTheta = blueTagPose.getRotation();


            this.blueOffsetX = blueOffsetArray[0];
            this.blueOffsetY = blueOffsetArray[1];
            this.blueOffsetThetaDegrees = blueOffsetArray[2];
            this.blueOffsetTheta = Rotation2d.fromDegrees(blueOffsetArray[2]);

            this.redTagPoseY = redTagPose.getY();
            this.redTagPoseThetaDegrees = redTagPose.getRotation().getDegrees();
            this.redTagPoseTheta = redTagPose.getRotation();
            this.redTagPoseX = redTagPose.getX();

            this.redOffsetX = redOffsetArray[0];
            this.redOffsetY = redOffsetArray[1];
            this.redOffsetThetaDegrees = redOffsetArray[2];
            this.redOffsetTheta = Rotation2d.fromDegrees(redOffsetArray[2]);
        }


    }

    public static AlignmentPose coralStationLeft = new AlignmentPose(aprilTagLayout.getTagPose(13).get().toPose2d(), null, aprilTagLayout.getTagPose(1).get().toPose2d(), null);
    public static AlignmentPose coralStationRight = new AlignmentPose(aprilTagLayout.getTagPose(12).get().toPose2d(), null, aprilTagLayout.getTagPose(2).get().toPose2d(), null);
    public static AlignmentPose processor = new AlignmentPose(aprilTagLayout.getTagPose(3).get().toPose2d(), null, aprilTagLayout.getTagPose(16).get().toPose2d(), null);
    public static AlignmentPose bargeOpposingView = new AlignmentPose(aprilTagLayout.getTagPose(4).get().toPose2d(), null, aprilTagLayout.getTagPose(15).get().toPose2d(), null);
    public static AlignmentPose bargeAllianceView = new AlignmentPose(aprilTagLayout.getTagPose(14).get().toPose2d(), null, aprilTagLayout.getTagPose(5).get().toPose2d(), null);
    public static AlignmentPose reefCloseLeft = new AlignmentPose(aprilTagLayout.getTagPose(19).get().toPose2d(), null, aprilTagLayout.getTagPose(6).get().toPose2d(), null);
    public static AlignmentPose reefCloseMid = new AlignmentPose(aprilTagLayout.getTagPose(18).get().toPose2d(), null, aprilTagLayout.getTagPose(7).get().toPose2d(), null);
    public static AlignmentPose reefCloseRight = new AlignmentPose(aprilTagLayout.getTagPose(17).get().toPose2d(), null, aprilTagLayout.getTagPose(8).get().toPose2d(), null);
    public static AlignmentPose reefFarRight = new AlignmentPose(aprilTagLayout.getTagPose(22).get().toPose2d(), null, aprilTagLayout.getTagPose(9).get().toPose2d(), null);
    public static AlignmentPose reefFarMid = new AlignmentPose(aprilTagLayout.getTagPose(21).get().toPose2d(), null, aprilTagLayout.getTagPose(10).get().toPose2d(), null);
    public static AlignmentPose reefFarLeft = new AlignmentPose(aprilTagLayout.getTagPose(20).get().toPose2d(), null, aprilTagLayout.getTagPose(11).get().toPose2d(), null);
}
