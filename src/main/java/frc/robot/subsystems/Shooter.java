// this file holds the subystem for the shooter mechanism
package frc.robot.subsystems;

import static frc.robot.Constants.IntakeConstants.*;
import static frc.robot.Constants.ShooterPIDConstants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    CANSparkMax m_topShooter;
    CANSparkMax m_bottomShooter;
    RelativeEncoder m_bottomEncoder;
    RelativeEncoder m_topEncoder;
    double setpoint;

    PIDController m_topController;
    SimpleMotorFeedforward topFeedforward;
    PIDController m_bottomController;
    SimpleMotorFeedforward bottomFeedforward;

    // constructor
    public Shooter() {
        // super(new PIDController(kP, kI, kD));
        m_topShooter = new CANSparkMax(kTopShooterID, MotorType.kBrushless);
        m_bottomShooter = new CANSparkMax(kBottomShooterID, MotorType.kBrushless);
        m_bottomEncoder = m_bottomShooter.getEncoder();
        m_topEncoder = m_topShooter.getEncoder();

        m_topController = new PIDController(kP, kI, kD);
        topFeedforward = new SimpleMotorFeedforward(kS, kV);
        m_topController.setTolerance(100, 10);

        m_bottomController = new PIDController(kP, kI, kD);
        bottomFeedforward = new SimpleMotorFeedforward(kS, kV);
        m_bottomController.setTolerance(100, 10);

        // m_controller.setSetpoint(0);
        // Top and bottom shooter should both be going outwards given positive input.
        // m_bottomShooter.setInverted(false);
        m_topShooter.setInverted(false);
        m_topController.setSetpoint(2550);
        m_topController.setSetpoint(2150);
    }

    public boolean atSetpoint() {
        return m_topController.atSetpoint() && m_bottomController.atSetpoint();
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

    // public void goToSpeed() {
    //     if (m_controller.getSetpoint() == 0 && m_controller.atSetpoint()) {
    //         m_controller.reset();
    //     }
    //     m_bottomShooter.setVoltage(m_controller.calculate(getTopShooterSpeed(), setpoint) + feedforward.calculate(setpoint));
    //     m_bottomShooter.setVoltage(m_bottomController.calculate(getBottomShooterSpeed(), setpoint) + feedforward.calculate(setpoint));
    // }

    public void setSpeed(double speed) {
        m_topShooter.set(speed);
        m_bottomShooter.set(speed - .1);
    }

    public void setShooterVoltage(double volts) {
        m_topShooter.setVoltage(volts);
        m_bottomShooter.setVoltage(volts);
    }

    // method to stop shooter motors
    public void stopShooting() {
        m_topShooter.set(0);
        m_bottomShooter.set(0);
    }

    public void usePIDShooter() {
        m_topShooter.setVoltage(m_topController.calculate(m_topEncoder.getVelocity()) + topFeedforward.calculate(2500));
        m_bottomShooter.setVoltage(m_bottomController.calculate(m_bottomEncoder.getVelocity())  + bottomFeedforward.calculate(2150));
    }
}