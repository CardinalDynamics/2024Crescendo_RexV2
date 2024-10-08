// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.DrivetrainConstants.*;
import static frc.robot.Constants.RotatorConstants.kIntakeAngle;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPLTVController;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.proto.Kinematics;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;

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
  CANSparkMax m_leftFront;
  CANSparkMax m_leftRear;
  CANSparkMax m_rightFront;
  CANSparkMax m_rightRear;
  RelativeEncoder m_leftEncoder;
  RelativeEncoder m_rightEncoder;
  Boolean m_driveBackwards = false;
  DifferentialDrive differentialDrive;

  public final static AHRS m_navx = new AHRS(Port.kMXP);
  private final DifferentialDriveOdometry m_odometry;

  // public SysIdRoutine routine;

  // private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
  // private final MutableMeasure<Distance> m_distance = mutable(Meters.of(0));
  // private final MutableMeasure<Velocity<Distance>> m_velocity = mutable(MetersPerSecond.of(0));
  /*Constructor. This method is called when an instance of the class is created. This should generally be used to set up
   * member variables and perform any configuration or set up necessary on hardware.
   */
  public Drivetrain() {
    m_leftFront = new CANSparkMax(kLeftFrontID, MotorType.kBrushless);
    m_leftRear = new CANSparkMax(kLeftRearID, MotorType.kBrushless);
    m_rightFront = new CANSparkMax(kRightFrontID, MotorType.kBrushless);
    m_rightRear = new CANSparkMax(kRightRearID, MotorType.kBrushless);

    m_leftEncoder = m_leftFront.getEncoder();
    m_rightEncoder = m_rightFront.getEncoder();

    m_leftEncoder.setPositionConversionFactor(kDistanceFactor);
    m_leftEncoder.setPositionConversionFactor(kDistanceFactor);
    m_leftEncoder.setVelocityConversionFactor(kDistanceFactor/60);
    m_leftEncoder.setVelocityConversionFactor(kDistanceFactor/60);

    /*Sets current limits for the drivetrain motors. This helps reduce the likelihood of wheel spin, reduces motor heating
     *at stall (Drivetrain pushing against something) and helps maintain battery voltage under heavy demand */
    m_leftFront.setSmartCurrentLimit(kCurrentLimit);
    m_leftRear.setSmartCurrentLimit(kCurrentLimit);
    m_rightFront.setSmartCurrentLimit(kCurrentLimit);
    m_rightRear.setSmartCurrentLimit(kCurrentLimit);

    // Set the rear motors to follow the front motors.
    m_leftRear.follow(m_leftFront);
    m_rightRear.follow(m_rightFront);

    // Invert the left side so both side drive forward with positive motor outputs
    m_leftFront.setInverted(true);
    m_rightFront.setInverted(false);

    // Put the front motors into the differential drive object. This will control all 4 motors with
    // the rears set to follow the fronts
    m_drivetrain = new DifferentialDrive(m_leftFront, m_rightFront);
    m_odometry = new DifferentialDriveOdometry(m_navx.getRotation2d(), m_leftEncoder.getPosition(), m_rightEncoder.getPosition());
    m_navx.reset();
    resetEncoders();

    differentialDrive = new DifferentialDrive(m_leftFront, m_rightFront);

    ReplanningConfig config = new ReplanningConfig();
    AutoBuilder.configureRamsete(this::getPose, this::resetOdometry, this::getWheelSpeeds, this::driveRobotRelative, config, () -> false, this);

  }

  /*Method to control the drivetrain using arcade drive. Arcade drive takes a speed in the X (forward/back) direction
   * and a rotation about the Z (turning the robot about it's center) and uses these to control the drivetrain motors */
  public void arcadeDrive(double speed, double rotation) {
    m_drivetrain.arcadeDrive(speed, rotation);
  }

  public void setDriveVoltage(double voltage) {
    m_leftFront.setVoltage(voltage);
    m_rightFront.setVoltage(voltage);
    // SmartDashboard.putNumber("Front Left", leftFront.getEncoder().getVelocity());
    // SmartDashboard.putNumber("Back Left", leftRear.getEncoder().getVelocity());
    // SmartDashboard.putNumber("Front Right", rightFront.getEncoder().getVelocity());
    // SmartDashboard.putNumber("Back Right", rightRear.getEncoder().getVelocity());
  }

  public void setAltDriveVoltage(double voltage) {
    m_leftFront.setVoltage(voltage);
    m_rightFront.setVoltage(-voltage);
  }

  public void toggleDriveMode() {
    m_driveBackwards = !m_driveBackwards;
  }

  public Boolean getIsBackwards() {
    return m_driveBackwards;
  }

  public double getLeftEncoderPosition() {
    return m_leftEncoder.getPosition();
  }

  public double getRightEncoderPosition() {
    return m_rightEncoder.getPosition();
  }

  public double getRightEncoderVelocity() {
    return m_rightEncoder.getVelocity();
  }

  public double getLeftEncoderVelocity() {
    return m_leftEncoder.getVelocity();
  }

  public void resetEncoders() {
    m_leftEncoder.setPosition(0);
    m_rightEncoder.setPosition(0);
  }

  public double getGyroHeading() {
    return m_navx.getAngle();
  }

  public double getTurnRate() {
    return -m_navx.getRate();
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(m_navx.getRotation2d(), getLeftEncoderPosition(), getRightEncoderPosition(), pose);
  }

  public static void zeroHeading() {
    m_navx.reset();
  }

  public ChassisSpeeds getWheelSpeeds() {
    return new ChassisSpeeds(getLeftEncoderVelocity(), getRightEncoderVelocity(), Math.toRadians(getTurnRate()));
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftFront.setVoltage(leftVolts);
    m_rightFront.setVoltage(rightVolts);
    
  }

  public RelativeEncoder getLeftEncoder() {
    return m_leftEncoder;
  }

  public RelativeEncoder getRightEncoder() {
    return m_rightEncoder;
  }

  public AHRS getGyro() {
    return m_navx;
  }

  public DifferentialDriveOdometry getOdometry() {
    return m_odometry;
  }

  public void setMaxOutput(double maxOutput) {
    differentialDrive.setMaxOutput(maxOutput);
  }

  public void driveRobotRelative(ChassisSpeeds chassisSpeeds) {
    DifferentialDriveWheelSpeeds wheelSpeeds = kKinematics.toWheelSpeeds(chassisSpeeds);
    m_drivetrain.tankDrive(wheelSpeeds.leftMetersPerSecond, wheelSpeeds.rightMetersPerSecond);
  }

  @Override
  public void periodic() {
    /*This method will be called once per scheduler run. It can be used for running tasks we know we want to update each
     * loop such as processing sensor data. Our drivetrain is simple so we don't have anything to put here */
    m_odometry.update(m_navx.getRotation2d(), m_leftEncoder.getPosition(), m_rightEncoder.getPosition());
    SmartDashboard.putNumber("Left Encoder Meters", getLeftEncoderPosition());
    SmartDashboard.putNumber("Right Encoder Meters", getRightEncoderPosition());
    SmartDashboard.putNumber("Gyro Heading", getGyroHeading());
    SmartDashboard.putNumber("Left Encoder Velo", getLeftEncoderVelocity());
    SmartDashboard.putNumber("Right Encoder Velo", getRightEncoderVelocity());
  }
}