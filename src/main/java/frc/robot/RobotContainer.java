// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ExampleCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.generated.TunerConstantsBOW;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.LEDSubsystem;
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
  CoralIntake m_intake = new CoralIntake();
  Elevator m_elevator = new Elevator();
  public final LEDSubsystem m_ledSubsystem = new LEDSubsystem();

  private final CommandXboxController joystick = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);

  public final CommandSwerveDrivetrain drivetrain;

  private final DigitalInput drivetrainDIO = new DigitalInput(0);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.

    // Determine which drivetrain we are using
    if (drivetrainDIO.get()) {
      drivetrain = TunerConstants.createDrivetrain();
    } else {
      drivetrain = TunerConstantsBOW.createDrivetrain();
    }

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
                        -joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with
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
    // speed limiter button that slows the speed down if needed
    joystick
        .rightBumper()
        .onTrue(Commands.runOnce(() -> drivetrain.getPigeon2().setYaw(Degrees.of(180))));
    joystick
        .leftTrigger()
        .whileTrue(
            Commands.startEnd(
                () -> MaxSpeed = TunerConstantsBOW.kSpeedAt12Volts.in(MetersPerSecond) * 0.25,
                () -> MaxSpeed = TunerConstantsBOW.kSpeedAt12Volts.in(MetersPerSecond) * 0.5));

    drivetrain.registerTelemetry(logger::telemeterize);

    joystick.povLeft().onTrue(Commands.run(() -> drivetrain.shiftAlign(true)).withTimeout(0.5));
    joystick.povRight().onTrue(Commands.run(() -> drivetrain.shiftAlign(false)).withTimeout(0.5));

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

    m_elevator.setDefaultCommand(m_elevator.run(() -> m_elevator.setMotorToPIDCalc()));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is
    // pressed,
    // cancelling on release.
    // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());

    m_driverController.x().whileTrue(m_elevator.cm_elevatorMovement(0.2));
    m_driverController.y().whileTrue(m_elevator.cm_elevatorMovement(-0.2));

    operator.leftBumper().whileTrue(m_intake.cm_intakeCoral(0.25));
    operator.rightBumper().whileTrue(m_intake.cm_intakeCoral(-0.1));
    // operator.x().whileTrue(fullSequence(BotState.DEFAULT));
    // operator.y().whileTrue(fullSequence(BotState.INTAKECORAL));
    // operator.a().whileTrue(fullSequence(BotState.REEF));
    // operator.b().whileTrue(fullSequence(BotState.CLIMB));
    // operator.leftBumper().whileTrue(fullSequence(BotState.EJECT));

    // For levels one, two, and three respectiveley
    operator.a().onTrue(m_elevator.cm_setElevatorState("Default"));
    operator
        .b()
        .whileTrue(
            new SequentialCommandGroup(
                    m_elevator.cm_setElevatorState("L2").until(() -> m_elevator.isAtPosition()),
                    m_intake.cm_intakeCoral(0.25))
                .withTimeout(5));
    operator.x().whileTrue(m_elevator.cm_setElevatorState("L3"));
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
    return new PathPlannerAuto("Forward");
  }
}
