// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.cm_AlgaeRemoval;
import frc.robot.commands.cm_FullSequence;
import frc.robot.extensions.StateMachine;
import frc.robot.extensions.StateMachine.BotState;
import frc.robot.generated.TunerConstants;
import frc.robot.generated.TunerConstantsBOW;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.Vision;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  public double MaxSpeed =
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

  // Subsystems
  private final Telemetry logger = new Telemetry(MaxSpeed);
  public final Vision m_vision = new Vision();
  public final CoralIntake m_coralIntake = new CoralIntake();
  public final Elevator m_elevator = new Elevator();
  public final AlgaeIntake m_AlgaeIntake = new AlgaeIntake();
  public final LEDSubsystem m_ledSubsystem = new LEDSubsystem();

  public final CommandSwerveDrivetrain drivetrain;
  private final DigitalInput drivetrainDIO = new DigitalInput(0);

  // Controllers
  private final CommandXboxController joystick = new CommandXboxController(0);
  private final CommandGenericHID operatorBoard = new CommandGenericHID(1);
  private final CommandXboxController operator = new CommandXboxController(2);
  private final CommandXboxController sysid = new CommandXboxController(3);
  private final SendableChooser<Command> autoChooser;

  // Commands
  private final cm_FullSequence cmd_FullSequenceL1,
      cmd_FullSequenceL2,
      cmd_FullSequenceL3,
      cmd_FullSequenceL4,
      cmd_HumanIntake;
  private final cm_AlgaeRemoval cmd_AlgaeRemoval;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    cmd_FullSequenceL1 =
        new cm_FullSequence(BotState.L1, m_elevator, m_coralIntake, m_ledSubsystem);
    cmd_FullSequenceL2 =
        new cm_FullSequence(BotState.L2, m_elevator, m_coralIntake, m_ledSubsystem);
    cmd_FullSequenceL3 =
        new cm_FullSequence(BotState.L3, m_elevator, m_coralIntake, m_ledSubsystem);
    cmd_FullSequenceL4 =
        new cm_FullSequence(BotState.L4, m_elevator, m_coralIntake, m_ledSubsystem);
    cmd_HumanIntake =
        new cm_FullSequence(BotState.INTAKECORAL, m_elevator, m_coralIntake, m_ledSubsystem);

    cmd_AlgaeRemoval = new cm_AlgaeRemoval(m_elevator, m_coralIntake);

    // Determine which drivetrain we are using
    if (drivetrainDIO.get()) {
      drivetrain = TunerConstants.createDrivetrain();
    } else {
      drivetrain = TunerConstantsBOW.createDrivetrain();
    }

    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.applyRequest(
            () ->
                drive
                    .withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with
                    // negative Y
                    // (forward)
                    .withVelocityY(
                        -joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
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
        .leftTrigger()
        .whileTrue(
            Commands.startEnd(
                () -> MaxSpeed = TunerConstantsBOW.kSpeedAt12Volts.in(MetersPerSecond) * 0.25,
                () -> MaxSpeed = TunerConstantsBOW.kSpeedAt12Volts.in(MetersPerSecond) * 0.5));

    drivetrain.registerTelemetry(logger::telemeterize);

    // m_elevator.setDefaultCommand(m_elevator.run(() -> m_elevator.setBrake()));

    autoChooser = AutoBuilder.buildAutoChooser();

    joystick.povLeft().onTrue(Commands.run(() -> drivetrain.shiftAlign(true)).withTimeout(0.5));
    joystick.povRight().onTrue(Commands.run(() -> drivetrain.shiftAlign(false)).withTimeout(0.5));

    joystick
        .povDown()
        .whileTrue(Commands.run(() -> drivetrain.pidAutoAlign(new Pose2d(3, 7, new Rotation2d()))));

    // Configure the trigger bindings
    configureBindings();
    SmartDashboard.putData(m_vision);
    SmartDashboard.putData(m_elevator);
    SmartDashboard.putData(m_coralIntake);
    SmartDashboard.putData("Auto Chooser", autoChooser);
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
    // Trigger binding example
    /* new Trigger(m_exampleSubsystem::exampleCondition)
    .onTrue(new ExampleCommand(m_exampleSubsystem)); */

    // operator board bindings
    // manually moves elevator down
    operatorBoard.button(1).whileTrue(m_elevator.cm_moveElevator(-0.1));
    // n/a for now... not sure what i want to do with this just yet (likely climber)
    operatorBoard.button(2).onTrue(m_coralIntake.cm_runCoralPivotMotor(-0.1));
    operatorBoard.button(3).whileTrue(m_elevator.cm_moveElevator(0.1));
    // manually moves elevator up
    operatorBoard
        .button(4)
        .whileTrue(
            m_coralIntake.cm_runCoralPivotMotor(0.1)); // Change this to run the pivot for now
    // n/a for now... not sure what i want to do with this just yet (likely climber)
    // ejects game piece (coral for now)
    operatorBoard.button(5).whileTrue(cmd_AlgaeRemoval);
    // goes to default
    operatorBoard.button(6).onTrue(StateMachine.setGlobalState(BotState.DEFAULT).andThen());
    // algae intake
    operatorBoard
        .button(7)
        .onTrue((m_AlgaeIntake.runOnce(() -> m_AlgaeIntake.cm_intakeAlgae(-0.5))));
    // shoots processor
    operatorBoard
        .button(10)
        .onTrue((m_AlgaeIntake.runOnce(() -> m_AlgaeIntake.cm_intakeAlgae(0.5))));

    // coral intake
    operatorBoard.button(9).onTrue(cmd_HumanIntake);

    // shoots L4
    operatorBoard.button(8).onTrue(cmd_FullSequenceL4);
    // shoots L3
    operatorBoard.button(12).onTrue(cmd_FullSequenceL3);
    // shoots L2
    operatorBoard.button(13).onTrue(cmd_FullSequenceL2);
    // shoots L1
    operatorBoard.button(14).onTrue(cmd_FullSequenceL1); // Change this to run full sequence

    // Test operator controls
    operator.povUp().whileTrue(m_coralIntake.cm_intakeCoral(.20));
    operator.povDown().whileTrue(m_coralIntake.cm_intakeCoral(-.20));

    // SysID test controls
    sysid.povRight().onTrue(Commands.runOnce(SignalLogger::start));
    sysid.povLeft().onTrue(Commands.runOnce(SignalLogger::stop));

    sysid.y().whileTrue(m_coralIntake.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    sysid.a().whileTrue(m_coralIntake.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));

    sysid.b().whileTrue(m_coralIntake.sysIdDynamic(SysIdRoutine.Direction.kForward));
    sysid.x().whileTrue(m_coralIntake.sysIdDynamic(SysIdRoutine.Direction.kReverse));
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
    return autoChooser.getSelected();
  }
}
