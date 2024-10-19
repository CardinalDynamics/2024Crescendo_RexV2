// this file holds the subystem for the feeder/intake mechanism

package frc.robot.subsystems;

import static frc.robot.Constants.IntakeConstants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    CANSparkMax m_intake;
    DigitalInput sensor;

    // constructor
    public Intake () {
        m_intake = new CANSparkMax(kIntakeID, MotorType.kBrushless);

        // Intake motor should run inwards when given a positive input
        m_intake.setInverted(true);
        sensor = new DigitalInput(1);
    }

    // defining method to start the intake motor
    public void intakeNote() {
        m_intake.set(1.0);
    }

    public void outtakeNote() {
        m_intake.set(-.8);
    }

    // defining method to stop the intake motor
    public void stopIntake() {
        m_intake.set(0);
    }

    public boolean getNote() {
        return sensor.get();
    }

    public void periodic() {
        SmartDashboard.setDefaultBoolean("have note?", sensor.get());
    }
}