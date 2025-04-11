// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonVision extends SubsystemBase {
  /** Creates a new PhotonVision. */
  PhotonCamera camera;
  Transform2d getCamera;

  public PhotonVision(String name, Transform2d getCameraPosition) {
    camera = new PhotonCamera(null);
    getCamera = getCameraPosition;

  }

  public void getLatestTarget() { // gets the latest result from PhotonVision
    camera.getLatestResult();

  }

  public void getBestTarget(){ // gets the best target that the robot can go to
    PhotonTrackedTarget target = camera.getBestTarget();
  }




  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
