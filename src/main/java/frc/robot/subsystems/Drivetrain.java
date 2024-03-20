// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.DrivetrainConstants.*;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

/* This class declares the subsystem for the robot drivetrain if controllers are connected via CAN. Make sure to go to
 * RobotContainer and uncomment the line declaring this subsystem and comment the line for PWMDrivetrain.
 *
 * The subsystem contains the objects for the hardware contained in the mechanism and handles low level logic
 * for control. Subsystems are a mechanism that, when used in conjuction with command "Requirements", ensure
 * that hardware is only being used by 1 command at a time.
 */
public class Drivetrain extends SubsystemBase {
  /*Class member variables. These variables represent things the class needs to keep track of and use between
  different method calls. */
  DifferentialDrive m_drivetrain;
  CANSparkMax leftFront;
  CANSparkMax leftRear;
  CANSparkMax rightFront;
  CANSparkMax rightRear;
  RelativeEncoder m_leftEncoder;
  RelativeEncoder m_rightEncoder;

  public SysIdRoutine routine;

  private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
  private final MutableMeasure<Distance> m_distance = mutable(Meters.of(0));
  private final MutableMeasure<Velocity<Distance>> m_velocity = mutable(MetersPerSecond.of(0));
  /*Constructor. This method is called when an instance of the class is created. This should generally be used to set up
   * member variables and perform any configuration or set up necessary on hardware.
   */
  public Drivetrain() {
    leftFront = new CANSparkMax(kLeftFrontID, MotorType.kBrushless);
    leftRear = new CANSparkMax(kLeftRearID, MotorType.kBrushless);
    rightFront = new CANSparkMax(kRightFrontID, MotorType.kBrushless);
    rightRear = new CANSparkMax(kRightRearID, MotorType.kBrushless);

    m_leftEncoder = leftFront.getEncoder();
    m_rightEncoder = rightFront.getEncoder();

    /*Sets current limits for the drivetrain motors. This helps reduce the likelihood of wheel spin, reduces motor heating
     *at stall (Drivetrain pushing against something) and helps maintain battery voltage under heavy demand */
    leftFront.setSmartCurrentLimit(kCurrentLimit);
    leftRear.setSmartCurrentLimit(kCurrentLimit);
    rightFront.setSmartCurrentLimit(kCurrentLimit);
    rightRear.setSmartCurrentLimit(kCurrentLimit);

    // Set the rear motors to follow the front motors.
    leftRear.follow(leftFront);
    rightRear.follow(rightFront);

    // Invert the left side so both side drive forward with positive motor outputs
    leftFront.setInverted(true);
    rightFront.setInverted(false);

    // Put the front motors into the differential drive object. This will control all 4 motors with
    // the rears set to follow the fronts
    m_drivetrain = new DifferentialDrive(leftFront, rightFront);

    routine = new SysIdRoutine(new SysIdRoutine.Config(), new SysIdRoutine.Mechanism(
      (Measure<Voltage> volts) -> {
        leftFront.setVoltage(volts.in(Volts));
        rightFront.setVoltage(volts.in(Volts));
      },
      log -> {
        log.motor("drive-left").voltage(
          m_appliedVoltage.mut_replace(
            leftFront.get() * RobotController.getBatteryVoltage(), Volts))
          .linearPosition(m_distance.mut_replace(m_leftEncoder.getPosition(), Meters))
          .linearVelocity(
            m_velocity.mut_replace(m_leftEncoder.getVelocity(), MetersPerSecond));

        log.motor("drive-right").voltage(
          m_appliedVoltage.mut_replace(
            rightFront.get() * RobotController.getBatteryVoltage(), Volts))
          .linearPosition(m_distance.mut_replace(m_rightEncoder.getPosition(), Meters))
          .linearVelocity(
            m_velocity.mut_replace(m_rightEncoder.getVelocity(), MetersPerSecond));
      },
      this
    ));
  }

  /*Method to control the drivetrain using arcade drive. Arcade drive takes a speed in the X (forward/back) direction
   * and a rotation about the Z (turning the robot about it's center) and uses these to control the drivetrain motors */
  public void arcadeDrive(double speed, double rotation) {
    m_drivetrain.arcadeDrive(speed, rotation);
  }

  @Override
  public void periodic() {
    /*This method will be called once per scheduler run. It can be used for running tasks we know we want to update each
     * loop such as processing sensor data. Our drivetrain is simple so we don't have anything to put here */
  }
}