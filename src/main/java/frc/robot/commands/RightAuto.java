package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Rotator;
import frc.robot.subsystems.Shooter;
import static frc.robot.Constants.IntakeConstants.*;

public class RightAuto extends SequentialCommandGroup{
    public RightAuto(Drivetrain drivetrain, Intake intake, Rotator rotator, Shooter shooter) {
        ParallelCommandGroup driveAndIntake = new ParallelCommandGroup(
            (new RunCommand(() -> drivetrain.arcadeDrive(-.5, 0), drivetrain)).withTimeout(.8),
            new RunCommand(() -> intake.intakeNote(), intake).withTimeout(1)
        );
        addCommands(
            new InstantCommand(() -> rotator.setRotatorSpeed(-.25), rotator),
            new WaitCommand(2.0),
            new InstantCommand(() -> rotator.setRotatorSpeed(0), rotator),
            (new WaitCommand(0.1)),

            new RunCommand(() -> shooter.setSpeed(kShooterSpeed), shooter).withTimeout(1),
            new WaitCommand(0),
            new InstantCommand(() -> intake.intakeNote(), intake),
            new WaitCommand(1),
            new InstantCommand(() -> shooter.stopShooting(), shooter),
            new InstantCommand(() -> intake.stopIntake(), intake),

            (new RunCommand(() -> drivetrain.arcadeDrive(-.5, .41), drivetrain)).withTimeout(1),
            driveAndIntake,
            // new InstantCommand(() -> intake.outtakeNote(), intake),
            // new WaitCommand(.1),
            new InstantCommand(() -> intake.stopIntake(), intake),

            (new RunCommand(() -> drivetrain.arcadeDrive(.5, 0), drivetrain)).withTimeout(1),
            (new RunCommand(() -> drivetrain.arcadeDrive(.5, -.41), drivetrain)).withTimeout(1),
            (new RunCommand(() -> drivetrain.arcadeDrive(.5, 0), drivetrain)).withTimeout(.2),
            new InstantCommand(() -> drivetrain.setDriveVoltage(0), drivetrain),

            new RunCommand(() -> shooter.setSpeed(kShooterSpeed), shooter).withTimeout(1),
            new WaitCommand(0),
            new InstantCommand(() -> intake.intakeNote(), intake),
            new WaitCommand(1),
            new InstantCommand(() -> shooter.stopShooting(), shooter),
            new InstantCommand(() -> intake.stopIntake(), intake)
        );

        addRequirements(drivetrain, intake, rotator, shooter);
    }
}