
package frc.robot.subsystems.Vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

public class VisionConstants {
    public static AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

    public static String camera1Name = "Camera1";
    public static String camera2Name = "Camera2"; // Turret-mounted camera

    // Robot to camera transforms
    // Camera 1: Fixed to robot chassis, 7.64" forward, 8.25" left, facing left (90° CCW yaw)
    public static Transform3d robotToCamera1 = new Transform3d(
        Units.inchesToMeters(7.64),    // +X: forward from robot center
        Units.inchesToMeters(8.25),    // +Y: left from robot center
        Units.inchesToMeters(9.25),    // +Z: height above robot base
        new Rotation3d(0, Units.degreesToRadians(-23), Units.degreesToRadians(90))
    );

    // Camera 2: Mounted on turret - this transform is from turret center to camera
    // The actual robot-to-camera transform is calculated dynamically based on turret angle
    public static Transform3d turretToCamera2 = new Transform3d(
        Units.inchesToMeters(0.6),  // Forward from turret center
        -Units.inchesToMeters(4.65),   // Lateral offset
        Units.inchesToMeters(0),  // Height above turret
        new Rotation3d(Units.degreesToRadians(2.81), Units.degreesToRadians(-30), 0)
    );

    // Turret position on robot (for calculating camera 2 position)
    public static Transform3d robotToTurretCenter = new Transform3d(
        Units.inchesToMeters(7.25),   // Forward from robot center
        Units.inchesToMeters(0.0),   // Lateral from robot center
        Units.inchesToMeters(17.25),  // Height above ground
        new Rotation3d(0, 0, 0)
    );

    // Which cameras are mounted on the turret (moving)
    public static boolean[] cameraOnTurret = new boolean[] {
        false, // Camera 1 - fixed to chassis
        true   // Camera 2 - on turret
    };

    // Basic filtering thresholds
    public static double maxAmbiguity = 0.3;
    public static double maxZError = 0.75;

    // Standard deviation baselines, for 1 meter distance and 1 tag
    // Lower = trust vision more over odometry.
    // At Xm distance with N tags, effective stddev = baseline * (X^2 / N)
    public static double linearStdDevBaseline = 0.01; // Meters - very high vision trust
    public static double angularStdDevBaseline = 0.04;  // Radians - very high vision trust

    // (Adjust to trust some cameras more than others)
    public static double[] cameraStdDevFactors = new double[] {
            1.0, // Camera 1
            1.0  // Camera 2
    };
}