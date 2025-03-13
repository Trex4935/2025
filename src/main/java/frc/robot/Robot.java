// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.extensions.LimelightHelpers;
import frc.robot.extensions.PhysicsSim;
import frc.robot.generated.TunerConstants;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  public static PowerDistribution solenoidSwitch = new PowerDistribution(1, ModuleType.kRev);
  // CANrange CANrange;
  private final RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    // CANrangeConfiguration configs = new CANrangeConfiguration();
    // CANrange = new CANrange(0);

    // Write these configs to the CANrange
    // CANrange.getConfigurator().apply(configs);
  }

  private double getYawInverted() {
    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
      return m_robotContainer.drivetrain.getPigeon2().getYaw().getValueAsDouble() + 180;
    } else {
      return m_robotContainer.drivetrain.getPigeon2().getYaw().getValueAsDouble();
    }
  }

  @Override
  public void robotInit() {
    SmartDashboard.setDefaultBoolean("Set Switchable Channel", false);
  }

  @Override
  public void robotPeriodic() {
    // var distance = CANrange.getDistance();

    // Refresh and print these values
    // System.out.println("Distance is " + distance.refresh().toString());

    CommandScheduler.getInstance().run();
    var driveState = m_robotContainer.drivetrain.getState();
    double headingDeg = getYawInverted();
    double omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);

    LimelightHelpers.SetRobotOrientation("limelight-bow", headingDeg, 0, 0, 0, 0, 0);
    var llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-bow");
    if (llMeasurement != null && llMeasurement.tagCount > 0 && omegaRps < 2.0) {
      m_robotContainer.drivetrain.addVisionMeasurement(
          llMeasurement.pose, Utils.fpgaToCurrentTime(llMeasurement.timestampSeconds));
    }
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {

    SignalLogger.setPath("/media/sda1/logs/");

    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    if (m_robotContainer.m_elevator.leftElevatorMotor.getPosition().getValueAsDouble()
        >= m_robotContainer.m_elevator.maxElevatorRotation) {
      m_robotContainer.MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 0.15;
    } else {
      m_robotContainer.MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 0.5;
    }
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
    PhysicsSim.getInstance().run();
  }
}
