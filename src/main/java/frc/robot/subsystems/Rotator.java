// this file holds the subystem for lifting and lowering the arm
package frc.robot.subsystems;

import static frc.robot.Constants.RotatorConstants.*;
import static frc.robot.Constants.RotatorPIDConstants.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Rotator extends SubsystemBase {
    CANSparkMax m_leftArm;
    CANSparkMax m_rightArm;
    RelativeEncoder m_leftEncoder;
    RelativeEncoder m_rightEncoder;
    double setpoint;
    PIDController m_controller;

    // constructor
    public Rotator() {
        m_leftArm = new CANSparkMax(kArmID, MotorType.kBrushless);
        m_rightArm = new CANSparkMax(kSecondArmID, MotorType.kBrushless);
        m_leftArm.setInverted(true);
        m_rightArm.setInverted(false);

        m_controller = new PIDController(kP, kI, kD);
        m_controller.setTolerance(.3);

        m_leftEncoder = m_leftArm.getEncoder();
        m_rightEncoder = m_rightArm.getEncoder();
        // divide by geardown ratio multiplly by degrees
        m_leftEncoder.setPositionConversionFactor((360/105));
        m_rightEncoder.setPositionConversionFactor((360/105));

        m_leftEncoder.setPosition(0);
        m_rightEncoder.setPosition(0);
        setpoint = 0;
    }

    public double getMeasurement() {
        return (m_leftEncoder.getPosition() + m_rightEncoder.getPosition()) / 2;
    }

    public boolean atSetpoint() {
        return m_controller.atSetpoint();
    }

    public void setRotatorVoltage(double voltage) {
        m_rightArm.setVoltage(voltage);
        m_leftArm.setVoltage(voltage);
    }

    public void setRotatorSpeed(double speed) {
        m_leftArm.set(speed);
        m_leftArm.set(speed);
    }
    
    // defining method to stop arm motors
    public void rotatorStop() {
        m_leftArm.set(0);
        m_rightArm.set(0);
    }

    public void zeroEncoder() {
        m_leftEncoder.setPosition(0);
        m_rightEncoder.setPosition(0);
    }

    public void setSetPoint(double setpoint) {
        this.setpoint = setpoint;
    }

    public void goToSetpoint() {
        if (m_controller.atSetpoint()) {
            m_controller.reset();
        }
        setRotatorVoltage(m_controller.calculate(getMeasurement(), setpoint));
    }

}