// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ExampleCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Vision;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private double MaxSpeed =
      TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 0.5; // adjust multipler for speed
  // speed
  private double MaxAngularRate =
      RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second
  // max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * 0.1)
          .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
          .withDriveRequestType(
              DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private final Telemetry logger = new Telemetry(MaxSpeed);
  Vision m_vision = new Vision();
  Intake m_intake = new Intake();
  Elevator m_elevator = new Elevator();

  private final CommandXboxController joystick = new CommandXboxController(0);

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.applyRequest(
            () ->
                drive
                    .withVelocityX(joystick.getLeftY() * MaxSpeed) // Drive forward with
                    // negative Y
                    // (forward)
                    .withVelocityY(
                        joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(
                        joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with
            // negative X (left)
            // if inversion for the 2nd joystick
            // broken change to pos -joseph
            ));

    joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    joystick
        .b()
        .whileTrue(
            drivetrain.applyRequest(
                () ->
                    point.withModuleDirection(
                        new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    // reset the field-centric heading on left bumper press
    joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

    drivetrain.registerTelemetry(logger::telemeterize);

    // Intake test code
    m_driverController.a().whileTrue(m_intake.intakeGo());
    m_driverController.b().whileTrue(m_intake.intakeDrop());

    // Configure the trigger bindings
    configureBindings();
    SmartDashboard.putData(m_vision);
    SmartDashboard.putData(m_elevator);
    SmartDashboard.putData(m_intake);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is
    // pressed,
    // cancelling on release.
    // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());

    m_driverController.x().whileTrue(m_elevator.cm_movementUp());
    m_driverController.y().whileTrue(m_elevator.cm_movementDown());
    m_driverController.a().whileTrue(m_intake.intakeGo());
    m_driverController.b().whileTrue(m_intake.intakeDrop());

    // Auto-drive commands
    m_driverController
        .back()
        .and(m_driverController.povUpLeft())
        .whileTrue(drivetrain.ppAutoDrive(Locations.reefFarLeft));
    m_driverController
        .back()
        .and(m_driverController.povUp())
        .whileTrue(drivetrain.ppAutoDrive(Locations.reefFarMid));
    m_driverController
        .back()
        .and(m_driverController.povUpRight())
        .whileTrue(drivetrain.ppAutoDrive(Locations.reefFarRight));
    m_driverController
        .back()
        .and(m_driverController.povDownLeft())
        .whileTrue(drivetrain.ppAutoDrive(Locations.reefCloseLeft));
    m_driverController
        .back()
        .and(m_driverController.povDown())
        .whileTrue(drivetrain.ppAutoDrive(Locations.reefCloseMid));
    m_driverController
        .back()
        .and(m_driverController.povDownRight())
        .whileTrue(drivetrain.ppAutoDrive(Locations.reefCloseRight));

    m_driverController
        .back()
        .and(m_driverController.povLeft())
        .whileTrue(drivetrain.ppAutoDrive(Locations.coralStationLeft));
    m_driverController
        .back()
        .and(m_driverController.povRight())
        .whileTrue(drivetrain.ppAutoDrive(Locations.coralStationRight));

        /*
    m_driverController
        .start()
        .and(m_driverController.povUp())
        .whileTrue(drivetrain.ppAutoDrive(Locations.halltag1, 49));
        */
    m_driverController
    .start().and(m_driverController.povUp()).whileTrue(drivetrain.ppSimple(new Pose2d(10, 3.6,new Rotation2d(-179))));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // This method loads the auto when it is called, however, it is recommended
    // to first load your paths/autos when code starts, then return the
    // pre-loaded auto/path
    return new PathPlannerAuto("Forward Backward Left Right");
  }
}