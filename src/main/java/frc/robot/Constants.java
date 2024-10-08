// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    // Port numbers for driver and operator gamepads. These correspond with the numbers on the USB
    // tab of the DriverStation
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
  }

  public static class DrivetrainConstants {
    // CAN IDs for drive motor controllers
    public static final int kLeftRearID = 8;
    public static final int kLeftFrontID = 12;
    public static final int kRightFrontID = 3;
    public static final int kRightRearID = 2;
    public static final double kDriveRatio = 8.46;
    public static final double kWheelRadius = 6;
    public static final double kDistanceFactor = 
      (Units.inchesToMeters(1 / (kDriveRatio * 2 * Math.PI * Units.inchesToMeters(kWheelRadius)) * 10));
    
    public static final double kTrackWidth = Units.inchesToMeters(23);
    public static final DifferentialDriveKinematics kKinematics = new DifferentialDriveKinematics(kTrackWidth);


    // Current limit for drivetrain motors
    public static final int kCurrentLimit = 60;

    public static final double kS = .26;
    public static final double kV = 450;
  }

  public static class IntakeConstants {
    // CAN IDs for intake/manipulator, shooter motor speed
    public static final int kTopShooterID = 6;
    public static final int kBottomShooterID = 5;
    public static final int kIntakeID = 4;
    public static final double kShooterSpeed = .45;
    // public static final double kBottomShooterSpeed = -.9;
  }

  public static class SolenoidConstants {
    // solenoid CAN ID
    public static final int kRotatorForward = 1;
    public static final int kRotatorReverse = 7;
    public static final int kPneumaticsHubID = 1;
  }

  public static class RotatorConstants {
    // Arm motorcontroller CAN IDs and motor speeds
    public static final int kArmID = 7;
    public static final int kSecondArmID = 9;
    // public static final double kArmSpeedUp = .5;
    // public static final double kArmSpeedDown = -.5;

    public static final double kPivotHeightInches = 9;
    public static final double kIntakeRadiusInches = 8.5;
    public static final double kSpeakerHeightInches = 78;
    public static final double kStartAngle = 77;
    public static final double kIntakeAngle = 0;
    public static final int kEncoderPort = 0;
  }

  public static class RotatorPIDConstants {
    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kS = 0;
    public static final double kG = 0;
    public static final double kV = 0;
    public static final double kTolerance = .3;

    // public static final double kArmUpSetPoint = 95;
    // public static final double kShootingPositionSetPoint = 0;
  }

  public static class ShooterPIDConstants {
    // public static final double kP = 0.001;
    public static final double kP = 0.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    // public static final double kV = 0.002;
    public static final double kV = 0.0;
    public static final double kS = 0;
  }

  public static class DrivePIDConstants {
    public static final double kP = 0.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
  }

  public static class LimelightConstants {
    public static final double kLimelightMountAngle = 67;
    public static final double kLimelightLenseHeight = 14.25;
    public static final double kTagHeight = 48.125;
  }
}