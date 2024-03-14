package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import static frc.robot.Constants.LimelightConstants.*;

// Uncomment when limelight is added
// public class Vision {

//     static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

//     public static double getDistanceFromSpeaker() {
//         double yOffsetAngle = table.getEntry("ty").getDouble(0);
//         return 
//             (48.125 - kLimelightLenseHeight)
//             / Math.tan(
//                 (kLimelightMountAngle + yOffsetAngle) * (3.1415926 / 180.0)
//             );
//     }
// }