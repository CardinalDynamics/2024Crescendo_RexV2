// this file holds the subystem for the shooter mechanism
package frc.robot.subsystems;

import static frc.robot.Constants.IntakeConstants.*;
import static frc.robot.Constants.ShooterPIDConstants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    CANSparkMax m_topShooter;
    CANSparkMax m_bottomShooter;
    RelativeEncoder m_bottomEncoder;
    RelativeEncoder m_topEncoder;
    double setpoint;
    PIDController m_controller;

    // constructor
    public Shooter() {
        // super(new PIDController(kP, kI, kD));
        m_topShooter = new CANSparkMax(kTopShooterID, MotorType.kBrushless);
        m_bottomShooter = new CANSparkMax(kBottomShooterID, MotorType.kBrushless);
        m_bottomEncoder = m_bottomShooter.getEncoder();
        m_topEncoder = m_topShooter.getEncoder();
        m_controller = new PIDController(kP, kI, kD);
        m_controller.setTolerance(100);
        m_controller.setSetpoint(0);
        // Top and bottom shooter should both be going outwards given positive input.
        m_bottomShooter.setInverted(false);
        m_topShooter.setInverted(false);
        setpoint = 0;
    }

    public boolean atSetpoint() {
        return m_controller.atSetpoint();
    }

    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    public double getBottomShooterSpeed() {
        if (m_bottomEncoder.getVelocity() < 10 && m_bottomEncoder.getVelocity() > -10) {
            return 0.0;
        }
        return m_bottomEncoder.getVelocity();
    }

    public double getTopShooterSpeed() {
        if (m_topEncoder.getVelocity() < 10 && m_topEncoder.getVelocity() > -10) {
            return 0.0;
        }
        return m_topEncoder.getVelocity();
    }

    public void goToSpeed() {
        if (m_controller.getSetpoint() == 0 && m_controller.atSetpoint()) {
            m_controller.reset();
        }
        m_bottomShooter.setVoltage(m_controller.calculate(getTopShooterSpeed(), setpoint));
        m_bottomShooter.setVoltage(m_controller.calculate(getBottomShooterSpeed(), setpoint));
    }

    public void setSpeed(double speed) {
        m_topShooter.set(speed);
        m_bottomShooter.set(speed - .1);
    }

    // method to stop shooter motors
    public void stopShooting() {
        m_topShooter.set(0);
        m_bottomShooter.set(0);
    }
}