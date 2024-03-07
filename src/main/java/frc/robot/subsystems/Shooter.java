// this file holds the subystem for the shooter mechanism
package frc.robot.subsystems;

import static frc.robot.Constants.IntakeConstants.*;
import static frc.robot.Constants.ShooterPIDConstants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

public class Shooter extends PIDSubsystem {
    CANSparkMax topShooter;
    CANSparkMax bottomShooter;
    RelativeEncoder m_bottomEncoder;
    RelativeEncoder m_topEncoder;
    double setpoint;

    // constructor
    public Shooter() {
        super(new PIDController(kP, kI, kD));
        getController().setTolerance(100);
        topShooter = new CANSparkMax(kTopShooterID, MotorType.kBrushless);
        bottomShooter = new CANSparkMax(kBottomShooterID, MotorType.kBrushless);
        m_bottomEncoder = bottomShooter.getEncoder();
        m_topEncoder = topShooter.getEncoder();

        // Top and bottom shooter should both be going outwards given positive input.
        bottomShooter.setInverted(true);
        topShooter.setInverted(false);
        setpoint = 0;
    }

    public double getMeasurement() {
        return
            (m_bottomEncoder.getVelocity()
            + m_topEncoder.getVelocity()) / 2;
    }

    public boolean atSetpoint() {
        return m_controller.atSetpoint();
    }

    public void useOutput(double output, double setpoint) {
        topShooter.setVoltage(output);
        topShooter.setVoltage(output);
    }

    // method to stop shooter motors
    public void stopShooting() {
        topShooter.set(0);
        bottomShooter.set(0);
    }
}