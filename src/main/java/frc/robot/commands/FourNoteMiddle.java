package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Rotator;
import frc.robot.subsystems.Shooter;

public class FourNoteMiddle extends SequentialCommandGroup {

    public FourNoteMiddle(Drivetrain drivetrain, Intake intake, Rotator rotator, Shooter shooter, Command path1, Command path2, Command path3, Command path4, Command path5, Command path6) {
        SequentialCommandGroup shootNote1 = new SequentialCommandGroup(
            new RunCommand(() -> shooter.setSpeed(IntakeConstants.kShooterSpeed), shooter).withTimeout(1),
            new WaitCommand(0),
            new InstantCommand(() -> intake.intakeNote(), intake),
            new WaitCommand(1),
            new InstantCommand(() -> shooter.stopShooting(), shooter),
            new InstantCommand(() -> intake.stopIntake(), intake)
        );

        SequentialCommandGroup shootNote2 = new SequentialCommandGroup(
            new RunCommand(() -> shooter.setSpeed(IntakeConstants.kShooterSpeed), shooter).withTimeout(1),
            new WaitCommand(0),
            new InstantCommand(() -> intake.intakeNote(), intake),
            new WaitCommand(1),
            new InstantCommand(() -> shooter.stopShooting(), shooter),
            new InstantCommand(() -> intake.stopIntake(), intake)
        );

        SequentialCommandGroup shootNote3 = new SequentialCommandGroup(
            new RunCommand(() -> shooter.setSpeed(IntakeConstants.kShooterSpeed), shooter).withTimeout(1),
            new WaitCommand(0),
            new InstantCommand(() -> intake.intakeNote(), intake),
            new WaitCommand(1),
            new InstantCommand(() -> shooter.stopShooting(), shooter),
            new InstantCommand(() -> intake.stopIntake(), intake)
        );

        SequentialCommandGroup shootNote4 = new SequentialCommandGroup(
            new RunCommand(() -> shooter.setSpeed(IntakeConstants.kShooterSpeed), shooter).withTimeout(1),
            new WaitCommand(0),
            new InstantCommand(() -> intake.intakeNote(), intake),
            new WaitCommand(1),
            new InstantCommand(() -> shooter.stopShooting(), shooter),
            new InstantCommand(() -> intake.stopIntake(), intake)
        );

        addCommands(
            shootNote1,
            new ParallelCommandGroup(path1, new InstantCommand(() -> intake.intakeNote(), intake)),
            new InstantCommand(() -> intake.stopIntake(), intake),
            path2
        );

        addCommands(
            shootNote2,
            new ParallelCommandGroup(path3, new InstantCommand(() -> intake.intakeNote(), intake)),
            new InstantCommand(() -> intake.stopIntake(), intake),
            path4
        );

        addCommands(
            shootNote3,
            new ParallelCommandGroup(path5, new InstantCommand(() -> intake.intakeNote(), intake)),
            new InstantCommand(() -> intake.stopIntake(), intake),
            path6,
            shootNote4
        );
    }
}
