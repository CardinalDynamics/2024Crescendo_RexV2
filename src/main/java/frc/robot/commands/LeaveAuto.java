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

public class LeaveAuto extends SequentialCommandGroup{
    public LeaveAuto(Drivetrain drivetrain) {
        addCommands(
            (new RunCommand(() -> drivetrain.arcadeDrive(-.5, 0), drivetrain)).withTimeout(1.5)
        );
        addRequirements(drivetrain);
    }
}