
package frc.robot.subsystems.Vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class VisionConstants {
    public static AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

    public static String camera1Name = "Camera1";

    // Robot to camera transforms
    public static Transform3d robotToCamera1 = new Transform3d(0, 0.0, 0.0, new Rotation3d(0, 0, 0)); // Example values, adjust as needed

    // Basic filtering thresholds
    public static double maxAmbiguity = 0.3;
    public static double maxZError = 0.75;

    // Standard deviation baselines, for 1 meter distance and 1 tag
    public static double linearStdDevBaseline = 0.02; // Meters
    public static double angularStdDevBaseline = 0.06; // Radians

    // (Adjust to trust some cameras more than others)
    public static double[] cameraStdDevFactors = new double[] {
            1.0, // Camera 1
    };
}