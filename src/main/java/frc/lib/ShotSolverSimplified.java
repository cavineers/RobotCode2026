package frc.lib;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public final class ShotSolverSimplified {

    // Lookup table: distance (meters) -> motor RPM
    private static final InterpolatingDoubleTreeMap distanceToRPM = new InterpolatingDoubleTreeMap();

    static {
        // TODO: Fill with real characterization data
        distanceToRPM.put(1.0, 750.0);
        distanceToRPM.put(2.0, 1000.0);
        distanceToRPM.put(3.0, 1250.0);
        distanceToRPM.put(4.0, 1500.0);
        distanceToRPM.put(5.0, 1750.0);
    }

    /**
     * Calculate angle to target (no velocity compensation).
     */
    public static Rotation2d getAngleToTarget(Pose2d robotPose, Pose2d targetPose) {
        double dx = targetPose.getX() - robotPose.getX();
        double dy = targetPose.getY() - robotPose.getY();
        return new Rotation2d(Math.atan2(dy, dx));
    }

    /**
     * Calculate angle to target with robot velocity compensation (lead target).
     * 
     * @param robotPose Robot position
     * @param robotVelocity Robot field-relative velocity
     * @param targetPose Target position
     * @param estimatedFlightTime Estimated time for projectile to reach target (seconds)
     * @return Angle to lead target position
     */
    public static Rotation2d getAngleToTargetWithVelocity(
            Pose2d robotPose,
            ChassisSpeeds robotVelocity,
            Pose2d targetPose,
            double estimatedFlightTime) {
        
        // Calculate lead target position
        double leadX = targetPose.getX() - robotVelocity.vxMetersPerSecond * estimatedFlightTime;
        double leadY = targetPose.getY() - robotVelocity.vyMetersPerSecond * estimatedFlightTime;
        
        // Calculate angle to lead target
        double dx = leadX - robotPose.getX();
        double dy = leadY - robotPose.getY();
        return new Rotation2d(Math.atan2(dy, dx));
    }

    /**
     * Get RPM for a given distance using linear interpolation.
     */
    public static double getRPMForDistance(double distanceMeters) {
        return distanceToRPM.get(distanceMeters);
    }

    /**
     * Get distance to target.
     */
    public static double getDistanceToTarget(Pose2d robotPose, Pose2d targetPose) {
        double dx = targetPose.getX() - robotPose.getX();
        double dy = targetPose.getY() - robotPose.getY();
        return Math.hypot(dx, dy);
    }

    /**
     * Get lead target position for visualization.
     * 
     * @param robotVelocity Robot field-relative velocity
     * @param targetPose Original target position
     * @param estimatedFlightTime Estimated time for projectile to reach target (seconds)
     * @return Lead target position
     */
    public static Pose2d getLeadTargetPose(
            ChassisSpeeds robotVelocity,
            Pose2d targetPose,
            double estimatedFlightTime) {
        
        double leadX = targetPose.getX() - robotVelocity.vxMetersPerSecond * estimatedFlightTime;
        double leadY = targetPose.getY() - robotVelocity.vyMetersPerSecond * estimatedFlightTime;
        
        return new Pose2d(leadX, leadY, targetPose.getRotation());
    }
}

