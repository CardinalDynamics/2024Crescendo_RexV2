// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.IntakeConstants.kShooterSpeed;
import static frc.robot.Constants.RotatorConstants.kStartAngle;

import java.io.IOException;
import java.nio.file.Path;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.LeaveAuto;
import frc.robot.commands.LeftAuto;
import frc.robot.commands.MiddleAuto;
import frc.robot.commands.RightAuto;
import frc.robot.commands.FourNoteMiddle;
import frc.robot.commands.RotateToTag;
import frc.robot.commands.SingleNoteAuto;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Rotator;
import frc.robot.subsystems.Shooter;
// import frc.robot.subsytems.Vision;
import frc.robot.subsystems.Vision;



/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final Drivetrain m_drivetrain = new Drivetrain();
  public final Intake m_intake = new Intake();
  // public so I can use smartdashboard
  public final Rotator m_rotator = new Rotator();
  public final Shooter m_shooter = new Shooter();
  public final Vision m_vision = new Vision();

  public final SendableChooser<Command> m_autoChooser = new SendableChooser<>();
  private final Command kMiddleAuto = new MiddleAuto(m_drivetrain, m_intake, m_rotator, m_shooter);
  private final Command kRightCurveAuto = new RightAuto(m_drivetrain, m_intake, m_rotator, m_shooter);
  private final Command kLeftCurveAuto = new LeftAuto(m_drivetrain, m_intake, m_rotator, m_shooter);
  private final Command kSingleNoteAuto = new SingleNoteAuto(m_intake, m_rotator, m_shooter);
  private final Command kDriveOnlyAuto = new LeaveAuto(m_drivetrain);
  private Command kFourNoteMiddle;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_operatorController = new CommandXboxController(
      OperatorConstants.kOperatorControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    try {
      kFourNoteMiddle = new FourNoteMiddle(m_drivetrain, m_intake, m_rotator, m_shooter,
      AutoBuilder.followPath(PathPlannerPath.fromPathFile("Mid-2")),
      AutoBuilder.followPath(PathPlannerPath.fromPathFile("2-Mid")),
      AutoBuilder.followPath(PathPlannerPath.fromPathFile("Mid-1")),
      AutoBuilder.followPath(PathPlannerPath.fromPathFile("1-Mid")),
      AutoBuilder.followPath(PathPlannerPath.fromPathFile("Mid-3")),
      AutoBuilder.followPath(PathPlannerPath.fromPathFile("3-Mid")));
      SmartDashboard.putBoolean("Paths Loaded", true);
    } catch (Exception e) {
      SmartDashboard.putString("error", e.getMessage());
      kFourNoteMiddle = kMiddleAuto;
      SmartDashboard.putBoolean("Paths Loaded", false);
    }

    

    m_autoChooser.setDefaultOption("SHOOT 1 NOTE ONLY", kSingleNoteAuto);
    m_autoChooser.addOption("Middle Auto", kMiddleAuto);
    m_autoChooser.addOption("Red Amp Side", kLeftCurveAuto);
    m_autoChooser.addOption("Red Source Side", kRightCurveAuto);
    m_autoChooser.addOption("Blue Source Side", kLeftCurveAuto);
    m_autoChooser.addOption("Blue Amp Side", kRightCurveAuto);
    m_autoChooser.addOption("LEAVE POINTS ONLY", kDriveOnlyAuto);
    m_autoChooser.addOption("4 note??", kFourNoteMiddle);
    m_autoChooser.addOption("Straight", new SequentialCommandGroup(
      new InstantCommand(() -> m_drivetrain.resetEncoders(), m_drivetrain),
      AutoBuilder.followPath(PathPlannerPath.fromPathFile("Straight"))));

    SmartDashboard.putData(m_autoChooser);
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    m_drivetrain.setDefaultCommand(new RunCommand(() -> m_drivetrain.arcadeDrive(
        m_driverController.getLeftY(), m_driverController.getRightX()), m_drivetrain));
    // m_drivetrain.setDefaultCommand(new RunCommand(() ->
    // m_drivetrain.setDriveVoltage(.27), m_drivetrain));
    m_driverController.a().whileTrue(new RotateToTag(m_drivetrain, m_vision));
    // m_shooter.setDefaultCommand(Commands.run(() -> m_shooter.goToSpeed(),
    // m_shooter));

    // m_rotator.setDefaultCommand(Commands.run(() -> m_rotator.goToSetpoint(),
    // m_rotator));
    m_rotator.setDefaultCommand(Commands.run(() -> m_rotator.setRotatorSpeed(0), m_rotator));
    m_shooter.setDefaultCommand(new RunCommand(() -> m_shooter.stopShooting(), m_shooter));
    // m_operatorController.rightTrigger().onTrue(new InstantCommand(() ->
    // m_shooter.setSetpoint(1700), m_shooter))
    // .onFalse(new InstantCommand(() -> m_shooter.setSetpoint(0), m_shooter));
    m_operatorController.rightTrigger().whileTrue(new RunCommand(() -> m_shooter.setSpeed(kShooterSpeed), m_shooter));
    m_operatorController.leftTrigger().whileTrue(new RunCommand(() -> m_shooter.setShooterVoltage(11), m_shooter));

    m_operatorController.leftBumper().onTrue(new RunCommand(() -> m_intake.intakeNote(), m_intake))
        .onFalse(new RunCommand(() -> m_intake.stopIntake(), m_intake));
    m_operatorController.rightBumper().onTrue(new RunCommand(() -> m_intake.outtakeNote(), m_intake))
        .onFalse(new RunCommand(() -> m_intake.stopIntake(), m_intake));
    // m_operatorController.rightBumper().onTrue(new RunCommand(() ->
    // m_shooter.setSpeed(-.1), m_shooter)).onFalse(new RunCommand(() ->
    // m_shooter.setSpeed(0), m_shooter));

    m_operatorController.a().onTrue(new InstantCommand(() -> m_rotator.setSetPoint(0)));
    m_operatorController.a().whileTrue(Commands.run(() -> m_rotator.usePIDoutput(), m_rotator));

    m_operatorController.b().onTrue(new InstantCommand(
        () -> m_rotator.setSetPoint(m_rotator.getAngleFromDistance(m_vision.getDistanceFromSpeaker()))));
    m_operatorController.b().whileTrue(Commands.run(() -> m_rotator.usePIDoutput(), m_rotator));

    m_operatorController.y().whileTrue(Commands.run(() -> m_rotator.setRotatorSpeed(.35), m_rotator));
    m_operatorController.x().whileTrue(Commands.run(() -> m_rotator.setRotatorSpeed(-.25), m_rotator));

    // m_operatorController.b().whileTrue(Commands.run(() ->
    // m_shooter.usePIDShooter(), m_shooter));

    // m_driverController.a().whileTrue(Commands.run(() ->
    // m_drivetrain.routine.quasistatic(SysIdRoutine.Direction.kForward)));
    // m_driverController.b().whileTrue(Commands.run(() ->
    // m_drivetrain.routine.quasistatic(SysIdRoutine.Direction.kReverse)));
    // m_driverController.y().whileTrue(Commands.run(() ->
    // m_drivetrain.routine.dynamic(SysIdRoutine.Direction.kForward)));
    // m_driverController.x().whileTrue(Commands.run(() ->
    // m_drivetrain.routine.dynamic(SysIdRoutine.Direction.kReverse)));
    // m_operatorController.b().onTrue(new InstantCommand(() ->
    // m_rotator.setSetPoint(kIntakeAngle), m_shooter)).onFalse(new
    // InstantCommand(() -> m_rotator.setSetPoint(kStartAngle)));

    // Uncomment when LL added
    // m_operatorController.a().onTrue(new InstantCommand(() ->
    // SmartDashboard.putNumber("Distance From Speaker",
    // Vision.getDistanceFromSpeaker())))
    // .onFalse(new InstantCommand(() -> SmartDashboard.putNumber("Distance From
    // Speaker", 0.0)));
    SmartDashboard.putNumber("Shooter Angle", m_rotator.getMeasurement());
    SmartDashboard.putNumber("Top Shooter Speed",
        (m_shooter.getTopShooterSpeed()));
    SmartDashboard.putNumber("Bottom Shooter Speed",
        (m_shooter.getBottomShooterSpeed()));
    SmartDashboard.putNumber("Shooter Angle", m_rotator.getMeasurement());
    SmartDashboard.putNumber("Rotation Setpoint", m_rotator.getSetpoint());
    SmartDashboard.putNumber("Distance From Speaker", m_vision.getDistanceFromSpeaker());
    SmartDashboard.putBoolean("Encoder connected", m_rotator.encoderConnected());

    SmartDashboard.putBoolean("Shooter Ready", m_shooter.getTopShooterSpeed() > 2500);
    // m_rotator.setPID(Shuffleboard.getTab("PID Constants").add("kP",
    // 0).getEntry().getDouble(0),
    // Shuffleboard.getTab("PID Constants").add("kI", 0).getEntry().getDouble(0),
    // Shuffleboard.getTab("PID Constants").add("kD", 0).getEntry().getDouble(0));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }
}