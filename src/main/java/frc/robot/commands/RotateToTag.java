package frc.robot.commands;

import static frc.robot.Constants.DrivePIDConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

public class RotateToTag extends Command{

    Drivetrain m_drive;
    Vision m_vision;
    PIDController controller;

    public RotateToTag(Drivetrain drive, Vision vision) {
        m_drive = drive;
        m_vision = vision;
        controller = new PIDController(kP, kI, kD);
        controller.setTolerance(.01);
    }

    public void initialize() {
        controller.setSetpoint(0);
    }

    public void execute() {
        m_drive.setAltDriveVoltage(controller.calculate(m_vision.getTx()));
    }

    public boolean isFinished() {
        return controller.atSetpoint();
    }

    public void interrupted() {
        m_drive.setDriveVoltage(0);
    }
}
