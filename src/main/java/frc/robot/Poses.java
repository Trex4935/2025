// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public class Poses {

    public static AprilTagFieldLayout aprilTagLayout =
    AprilTagFieldLayout.loadField(AprilTagFields.k2024Reefscape);





    public static class AlignmentPose {
        public Pose2d aprilTagPose;
        public double[] offset;
        
        public double offsetX, offsetY, offsetThetaDegrees;
        public Rotation2d offsetTheta;
        
        public double tagPoseX, tagPoseY, tagPoseThetaDegrees;
        public Rotation2d tagPoseTheta;

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
}
