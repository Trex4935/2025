package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.AlignmentLocations;
import frc.robot.AlignmentLocations.AlignmentPose;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.Supplier;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements Subsystem so it can easily
 * be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
  private static final double kSimLoopPeriod = 0.005; // 5 ms
  private Notifier m_simNotifier = null;
  private double m_lastSimTime;

  /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
  private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
  /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
  private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
  /* Keep track if we've ever applied the operator perspective before or not */
  private boolean m_hasAppliedOperatorPerspective = false;
  /* Swerve request to apply during robot-centric path following */
  private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds =
      new SwerveRequest.ApplyRobotSpeeds();

  /* Swerve requests to apply during SysId characterization */
  private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization =
      new SwerveRequest.SysIdSwerveTranslation();
  private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization =
      new SwerveRequest.SysIdSwerveSteerGains();
  private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization =
      new SwerveRequest.SysIdSwerveRotation();

  /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
  private final SysIdRoutine m_sysIdRoutineTranslation =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null, // Use default ramp rate (1 V/s)
              Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
              null, // Use default timeout (10 s)
              // Log state with SignalLogger class
              state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())),
          new SysIdRoutine.Mechanism(
              output -> setControl(m_translationCharacterization.withVolts(output)), null, this));

  /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
  private final SysIdRoutine m_sysIdRoutineSteer =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null, // Use default ramp rate (1 V/s)
              Volts.of(7), // Use dynamic voltage of 7 V
              null, // Use default timeout (10 s)
              // Log state with SignalLogger class
              state -> SignalLogger.writeString("SysIdSteer_State", state.toString())),
          new SysIdRoutine.Mechanism(
              volts -> setControl(m_steerCharacterization.withVolts(volts)), null, this));

  /*
   * SysId routine for characterizing rotation.
   * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
   * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
   */
  private final SysIdRoutine m_sysIdRoutineRotation =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              /* This is in radians per second², but SysId only supports "volts per second" */
              Volts.of(Math.PI / 6).per(Second),
              /* This is in radians per second, but SysId only supports "volts" */
              Volts.of(Math.PI),
              null, // Use default timeout (10 s)
              // Log state with SignalLogger class
              state -> SignalLogger.writeString("SysIdRotation_State", state.toString())),
          new SysIdRoutine.Mechanism(
              output -> {
                /* output is actually radians per second, but SysId only supports "volts" */
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                /* also log the requested output for SysId */
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
              },
              null,
              this));

  /* The SysId routine to test */
  private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

  PhoenixPIDController swervePIDx = new PhoenixPIDController(0.5, 0, 0);
  PhoenixPIDController swervePIDy = new PhoenixPIDController(0.5, 0, 0);
  PhoenixPIDController swervePIDtheta = new PhoenixPIDController(0.3, 0, 0);

  /**
   * Constructs a CTRE SwerveDrivetrain using the specified constants.
   *
   * <p>This constructs the underlying hardware devices, so users should not construct the devices
   * themselves. If they need the devices, they can access them through getters in the classes.
   *
   * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
   * @param modules Constants for each specific module
   */
  public CommandSwerveDrivetrain(
      SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?>... modules) {
    super(drivetrainConstants, modules);
    if (Utils.isSimulation()) {
      startSimThread();
    }

    configureAutoBuilder();
  }

  /**
   * Constructs a CTRE SwerveDrivetrain using the specified constants.
   *
   * <p>This constructs the underlying hardware devices, so users should not construct the devices
   * themselves. If they need the devices, they can access them through getters in the classes.
   *
   * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
   * @param odometryUpdateFrequency The frequency to run the odometry loop. If unspecified or set to
   *     0 Hz, this is 250 Hz on CAN FD, and 100 Hz on CAN 2.0.
   * @param modules Constants for each specific module
   */
  public CommandSwerveDrivetrain(
      SwerveDrivetrainConstants drivetrainConstants,
      double odometryUpdateFrequency,
      SwerveModuleConstants<?, ?, ?>... modules) {
    super(drivetrainConstants, odometryUpdateFrequency, modules);
    if (Utils.isSimulation()) {
      startSimThread();
    }
    configureAutoBuilder();
  }

  /**
   * Constructs a CTRE SwerveDrivetrain using the specified constants.
   *
   * <p>This constructs the underlying hardware devices, so users should not construct the devices
   * themselves. If they need the devices, they can access them through getters in the classes.
   *
   * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
   * @param odometryUpdateFrequency The frequency to run the odometry loop. If unspecified or set to
   *     0 Hz, this is 250 Hz on CAN FD, and 100 Hz on CAN 2.0.
   * @param odometryStandardDeviation The standard deviation for odometry calculation in the form
   *     [x, y, theta]ᵀ, with units in meters and radians
   * @param visionStandardDeviation The standard deviation for vision calculation in the form [x, y,
   *     theta]ᵀ, with units in meters and radians
   * @param modules Constants for each specific module
   */
  public CommandSwerveDrivetrain(
      SwerveDrivetrainConstants drivetrainConstants,
      double odometryUpdateFrequency,
      Matrix<N3, N1> odometryStandardDeviation,
      Matrix<N3, N1> visionStandardDeviation,
      SwerveModuleConstants<?, ?, ?>... modules) {
    super(
        drivetrainConstants,
        odometryUpdateFrequency,
        odometryStandardDeviation,
        visionStandardDeviation,
        modules);
    if (Utils.isSimulation()) {
      startSimThread();
    }
    configureAutoBuilder();
  }

  private void configureAutoBuilder() {
    try {
      var config = RobotConfig.fromGUISettings();
      AutoBuilder.configure(
          () -> getState().Pose, // Supplier of current robot pose
          this::resetPose, // Consumer for seeding pose against auto
          () -> getState().Speeds, // Supplier of current robot speeds
          // Consumer of ChassisSpeeds and feedforwards to drive the robot
          (speeds, feedforwards) ->
              setControl(
                  m_pathApplyRobotSpeeds
                      .withSpeeds(speeds)
                      .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                      .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())),
          new PPHolonomicDriveController(
              // PID constants for translation
              new PIDConstants(10, 0, 0),
              // PID constants for rotation
              new PIDConstants(7, 0, 0)),
          config,
          // Assume the path needs to be flipped for Red vs Blue, this is normally the case
          () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
          this // Subsystem for requirements
          );
    } catch (Exception ex) {
      DriverStation.reportError(
          "Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
    }
  }

  /**
   * Returns a command that applies the specified control request to this swerve drivetrain.
   *
   * @param request Function returning the request to apply
   * @return Command to run
   */
  public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
    return run(() -> this.setControl(requestSupplier.get()));
  }

  /**
   * Runs the SysId Quasistatic test in the given direction for the routine specified by {@link
   * #m_sysIdRoutineToApply}.
   *
   * @param direction Direction of the SysId Quasistatic test
   * @return Command to run
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineToApply.quasistatic(direction);
  }

  /**
   * Runs the SysId Dynamic test in the given direction for the routine specified by {@link
   * #m_sysIdRoutineToApply}.
   *
   * @param direction Direction of the SysId Dynamic test
   * @return Command to run
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineToApply.dynamic(direction);
  }

  /**
   * Automatically drives to a pose
   *
   * @param pose The desired location to drive to
   * @return A command that drives to a location using PathPlanner
   */
  public Command ppAutoDrive(AlignmentPose pose) {
    // Default to Blue alliance if none is specified
    Alliance ally = DriverStation.getAlliance().orElse(Alliance.Blue);

    // Determine necessary tag pose
    Pose2d tagPose = (ally == Alliance.Blue ? pose.aprilTagPoseBlue : pose.aprilTagPoseRed);

    // Depending on the alliance, the offsets will either be added or subtracted
    double addOrSubtract = (ally == Alliance.Blue ? 1 : -1);
    Rotation2d yawOffset = (ally == Alliance.Blue ? new Rotation2d() : new Rotation2d(Math.PI));

    // Gets target values from the tag poses and the offset
    double targetX = tagPose.getX() + (addOrSubtract * pose.offsetX);
    double targetY = tagPose.getY() + (addOrSubtract * pose.offsetY);
    Rotation2d targetTheta = pose.offsetTheta.plus(yawOffset);

    // Create the target pose with the target translation and offset theta
    Pose2d targetPose = new Pose2d(targetX, targetY, targetTheta);

    // Create and return the auto-generated pathfinding command
    return AutoBuilder.pathfindToPose(targetPose, new PathConstraints(1, 1, 1, 1), 0);
  }


  public static Pose2d shiftPoseRobotCentricX(Pose2d pose, double distance) {
        Rotation2d heading = pose.getRotation();

        double deltaX = -distance * heading.getSin();
        double deltaY = distance * heading.getCos();

        Translation2d newTranslation = pose.getTranslation().plus(new Translation2d(deltaX, deltaY));

        return new Pose2d(newTranslation, heading);
    }

  /**
   * Automatically drives to the closest reef pose
   *
   * @return A command that drives to a location using PathPlanner
   */
  public Command ppAutoDriveNearest(double robotCentricOffsetX) {
    // Default to Blue alliance if none is specified
    Alliance ally = DriverStation.getAlliance().orElse(Alliance.Blue);

    // Add lists for poses and offsets
    List<Pose2d> poseList =
        ally == Alliance.Blue
            ? Arrays.asList(AlignmentLocations.reefPoseListBlue)
            : Arrays.asList(AlignmentLocations.reefPoseListRed);
    List<Double> xOffsetList = new ArrayList<>();
    List<Double> yOffsetList = new ArrayList<>();
    List<Rotation2d> thetaOffsetList = new ArrayList<>();

    // Determine necessary tag pose list and apply necessary offsets
    for (AlignmentPose pose : AlignmentLocations.reefTags) {
      xOffsetList.add(pose.offsetX);
      yOffsetList.add(pose.offsetY);
      thetaOffsetList.add(pose.offsetTheta);
    }

    Pose2d tagPose = this.getState().Pose.nearest(poseList);
    System.out.println(this.getState().Pose);

    double xOffset = xOffsetList.get(poseList.indexOf(tagPose));
    double yOffset = yOffsetList.get(poseList.indexOf(tagPose));
    Rotation2d heading = thetaOffsetList.get(poseList.indexOf(tagPose));

    // Depending on the alliance, the offsets will either be added or subtracted
    double addOrSubtract = (ally == Alliance.Blue ? 1 : -1);
    Rotation2d yawOffset = (ally == Alliance.Blue ? new Rotation2d() : new Rotation2d(Math.PI));

    // Gets target values from the tag poses and the offset
    double targetX = tagPose.getX() + (addOrSubtract * xOffset);
    double targetY = tagPose.getY() + (addOrSubtract * yOffset);
    Rotation2d targetTheta = heading.plus(yawOffset);

    // Create the target pose with the target translation and offset theta
    Pose2d targetPose = shiftPoseRobotCentricX(new Pose2d(targetX, targetY, targetTheta), robotCentricOffsetX);

    // Create and return the auto-generated pathfinding command
    return AutoBuilder.pathfindToPose(targetPose, new PathConstraints(1, 1, 1, 1), 0);
  }

  /**
   * Automatically drives to a pose
   *
   * @param pose The desired location to drive to
   * @param theSillyNumber A number to make this different from the above method, does nothing
   * @return A command that drives to a location using PathPlanner
   */
  public Command ppAutoDrive(AlignmentPose pose, double theSillyNumber) {

    // Determine necessary tag pose
    Pose2d tagPose = pose.aprilTagPose;

    // Gets target values from the tag poses and the offset
    double targetX = tagPose.getX() + pose.offsetX;
    double targetY = tagPose.getY() + pose.offsetY;

    // Create the target pose with the target translation and offset theta
    Pose2d targetPose = new Pose2d(targetX, targetY, pose.offsetTheta);

    // Create and return the auto-generated pathfinding command
    return AutoBuilder.pathfindToPose(targetPose, new PathConstraints(0.1, 0.1, 0.1, 0.1), 0);
  }

  public Command ppSimple(Pose2d goToPose) {
    return AutoBuilder.pathfindToPose(goToPose, new PathConstraints(0.5, 0.5, 0.5, 0.5), 0);
  }

  /** Shifts the robot left or right */
  public void shiftAlign(boolean shiftLeft) {
    // copied from documentation so might not be right
    final SwerveRequest.RobotCentric m_driveRequest = new SwerveRequest.RobotCentric();

    double shift = shiftLeft ? 1 : -1;

    // shift the robot
    this.setControl(m_driveRequest.withVelocityY(shift * 0.5));
  }

  @Override
  public void periodic() {
    /*
     * Periodically try to apply the operator perspective.
     * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
     * This allows us to correct the perspective in case the robot code restarts mid-match.
     * Otherwise, only check and apply the operator perspective if the DS is disabled.
     * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
     */
    if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
      DriverStation.getAlliance()
          .ifPresent(
              allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation);
                m_hasAppliedOperatorPerspective = true;
              });
    }
  }

  private void startSimThread() {
    m_lastSimTime = Utils.getCurrentTimeSeconds();

    /* Run simulation at a faster rate so PID gains behave more reasonably */
    m_simNotifier =
        new Notifier(
            () -> {
              final double currentTime = Utils.getCurrentTimeSeconds();
              double deltaTime = currentTime - m_lastSimTime;
              m_lastSimTime = currentTime;

              /* use the measured time delta, get battery voltage from WPILib */
              updateSimState(deltaTime, RobotController.getBatteryVoltage());
            });
    m_simNotifier.startPeriodic(kSimLoopPeriod);
  }

  /** Moves robot to a pose using a PID */
  public void pidAutoAlign(Pose2d targetPose) {
    // copied from documentation so might not be right
    final SwerveRequest.FieldCentric m_driveRequest =
        new SwerveRequest.FieldCentric()
            .withDeadband(0.1) // dont understand what these mean
            .withRotationalDeadband(0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo);

    swervePIDtheta.enableContinuousInput(-Math.PI, Math.PI);

    swervePIDx.setTolerance(0.05);
    swervePIDy.setTolerance(0.05);
    swervePIDtheta.setTolerance(Math.toRadians(0.1));

    Pose2d currentPose = this.getState().Pose;

    targetPose.getRotation().minus(currentPose.getRotation()).getRadians();
    // sets a control with the ratio between the x and y times the max speed

    double xApplied =
        -swervePIDx.calculate(currentPose.getX(), targetPose.getX(), this.getState().Timestamp);
    double yApplied =
        -swervePIDy.calculate(currentPose.getY(), targetPose.getY(), this.getState().Timestamp);
    double rotationApplied =
        swervePIDtheta.calculate(
            currentPose.getRotation().getDegrees(),
            targetPose.getRotation().getDegrees(),
            this.getState().Timestamp);

    this.setControl(
        m_driveRequest
            .withVelocityX(xApplied)
            .withVelocityY(yApplied)
            .withRotationalRate(rotationApplied));
  }
}
