package frc.robot.commands;

import static frc.robot.Constants.DrivePIDConstants.*;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

public class RotateToTag {

    Drivetrain m_drive;
    Vision m_vision;
    PIDController controller;

    public RotateToTag(Drivetrain drive, Vision vision) {
        m_drive = drive;
        m_vision = vision;
        controller = new PIDController(kP, kI, kD);
    }

    public void initialize() {

    }
}
