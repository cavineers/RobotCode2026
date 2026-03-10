
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
    // Camera 1: Fixed to robot chassis
    public static Transform3d robotToCamera1 = new Transform3d(
        Units.inchesToMeters(7.625), 
        -Units.inchesToMeters(8.25), 
        Units.inchesToMeters(7.25), 
        new Rotation3d(0, 0, Units.degreesToRadians(0))
    );

    // Camera 2: Mounted on turret - this transform is from turret center to camera
    // The actual robot-to-camera transform is calculated dynamically based on turret angle
    public static Transform3d turretToCamera2 = new Transform3d(
        Units.inchesToMeters(0.6),  // Forward from turret center
        Units.inchesToMeters(4.65),   // Lateral offset
        Units.inchesToMeters(12.0),  // Height above turret
        new Rotation3d(0, Units.degreesToRadians(20), 0) // Pitched down 15 degrees
    );

    // Turret position on robot (for calculating camera 2 position)
    public static Transform3d robotToTurretCenter = new Transform3d(
        Units.inchesToMeters(7.25),   // Forward from robot center
        Units.inchesToMeters(0.0),   // Lateral from robot center
        Units.inchesToMeters(15.91),  // Height above ground
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
    public static double linearStdDevBaseline = 0.02; // Meters
    public static double angularStdDevBaseline = 0.06; // Radians

    // (Adjust to trust some cameras more than others)
    public static double[] cameraStdDevFactors = new double[] {
            1.0, // Camera 1
            1.0  // Camera 2
    };
}