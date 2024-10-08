// this file holds the subystem for lifting and lowering the arm
package frc.robot.subsystems;

import static frc.robot.Constants.RotatorConstants.*;
import static frc.robot.Constants.RotatorPIDConstants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Rotator extends SubsystemBase {
    CANSparkMax m_leftArm;
    CANSparkMax m_rightArm;
    DutyCycleEncoder m_revEncoder;
    double setpoint;
    PIDController m_PIDcontroller;
    ArmFeedforward m_Feedforward;
    

    // constructor
    public Rotator() {
        m_leftArm = new CANSparkMax(kArmID, MotorType.kBrushless);
        m_rightArm = new CANSparkMax(kSecondArmID, MotorType.kBrushless);
        m_leftArm.setInverted(true);
        m_rightArm.setInverted(false);

        m_PIDcontroller = new PIDController(kP, kI, kD);
        m_PIDcontroller.setTolerance(kTolerance);

        m_Feedforward = new ArmFeedforward(kS, kG, kV);

        m_revEncoder = new DutyCycleEncoder(kEncoderPort);
        m_revEncoder.setConnectedFrequencyThreshold(500);
        m_revEncoder.reset();
    }

    public double getMeasurement() {
        // return (m_leftEncoder.getPosition() + m_rightEncoder.getPosition()) / 2;
        return 75.0 - (m_revEncoder.get() * 360.0);
    }

    public boolean encoderConnected() {
        return m_revEncoder.isConnected();
    }

    public boolean atSetpoint() {
        return m_PIDcontroller.atSetpoint();
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
        m_revEncoder.reset();
    }

    public void setSetPoint(double setpoint) {
        m_PIDcontroller.setSetpoint(setpoint);
    }

    public double getAngleFromDistance(double distance) {
        double pivotToSpeaker = kSpeakerHeightInches - kPivotHeightInches;
        double angle = Math.asin(kIntakeRadiusInches / -((
            kIntakeRadiusInches * (kIntakeRadiusInches * distance + pivotToSpeaker * 
                Math.sqrt(Math.pow(distance, 2) + Math.pow(pivotToSpeaker, 2) - Math.pow(kIntakeRadiusInches, 2)))
        )) / (- Math.pow(pivotToSpeaker, 2) + Math.pow(kIntakeRadiusInches, 2))
        );
        return angle * (180.0 / Math.PI);
    }

    public void usePIDoutput() {
        if (m_PIDcontroller.atSetpoint()) {
            m_PIDcontroller.reset();
        }
        setRotatorVoltage(m_PIDcontroller.calculate(getMeasurement(), setpoint) + m_Feedforward.calculate(getMeasurement() * Math.PI / 180.0, 0));
    }

    public double getSetpoint() {
        return setpoint;
    }

    public void setControl(double p, double i, double d) {
        m_PIDcontroller.setP(p);
        m_PIDcontroller.setI(i);
        m_PIDcontroller.setD(d);
    }
}