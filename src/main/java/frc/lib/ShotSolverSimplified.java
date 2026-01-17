package frc.lib;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public final class ShotSolverSimplified {

    /**
     * Shot parameters for a given distance.
     */
    public static class ShotParameters {
        public final double rpm;
        public final double pitchDegrees;

        public ShotParameters(double rpm, double pitchDegrees) {
            this.rpm = rpm;
            this.pitchDegrees = pitchDegrees;
        }
    }

    // Lookup tables: distance (meters) -> parameter
    private static final InterpolatingDoubleTreeMap distanceToRPM = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap distanceToPitch = new InterpolatingDoubleTreeMap();

    static {
        // TODO: Fill with real characterization data
        // These should be measured together at each distance
        distanceToRPM.put(1.0, 750.0);
        distanceToRPM.put(2.0, 1000.0);
        distanceToRPM.put(3.0, 1250.0);
        distanceToRPM.put(4.0, 1500.0);
        distanceToRPM.put(5.0, 1750.0);
        
        distanceToPitch.put(1.0, 30.0);  
        distanceToPitch.put(2.0, 35.0);  
        distanceToPitch.put(3.0, 40.0);  
        distanceToPitch.put(4.0, 45.0);  
        distanceToPitch.put(5.0, 50.0);  
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
     * @param robotPose           Robot position
     * @param robotVelocity       Robot field-relative velocity
     * @param targetPose          Target position
     * @param estimatedFlightTime Estimated time for projectile to reach target
     *                            (seconds)
     * @return Angle to lead target position
     */
    public static Rotation2d getAngleToTargetWithVelocity(
            Pose2d robotPose,
            ChassisSpeeds robotVelocity,
            Pose2d targetPose,
            double estimatedFlightTime) {

        // Calculate virtual target position
        // The projectile inherits the robot's velocity, so we need to aim behind
        // where the target appears to be (in the robot's reference frame)
        double leadX = targetPose.getX() - robotVelocity.vxMetersPerSecond * estimatedFlightTime;
        double leadY = targetPose.getY() - robotVelocity.vyMetersPerSecond * estimatedFlightTime;

        // Calculate angle to virtual target
        double dx = leadX - robotPose.getX();
        double dy = leadY - robotPose.getY();
        return new Rotation2d(Math.atan2(dy, dx));
    }

    /**
     * Get shot parameters (RPM and pitch) for a given distance.
     * Both values are interpolated together based on the same distance.
     */
    public static ShotParameters getShotParameters(double distanceMeters) {
        double rpm = distanceToRPM.get(distanceMeters);
        double pitch = distanceToPitch.get(distanceMeters);
        return new ShotParameters(rpm, pitch);
    }

    /**
     * Get RPM for a given distance using linear interpolation.
     */
    public static double getRPMForDistance(double distanceMeters) {
        return distanceToRPM.get(distanceMeters);
    }

    /**
     * Get pitch angle (degrees) for a given distance using linear interpolation.
     */
    public static double getPitchForDistance(double distanceMeters) {
        return distanceToPitch.get(distanceMeters);
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
     * @param robotVelocity       Robot field-relative velocity
     * @param targetPose          Original target position
     * @param estimatedFlightTime Estimated time for projectile to reach target
     *                            (seconds)
     * @return Virtual target position (aim point)
     */
    public static Pose2d getLeadTargetPose(
            ChassisSpeeds robotVelocity,
            Pose2d targetPose,
            double estimatedFlightTime) {

        // The projectile inherits the robot's velocity, so we need to aim behind
        // where the target appears to be (in the robot's reference frame)
        double leadX = targetPose.getX() - robotVelocity.vxMetersPerSecond * estimatedFlightTime;
        double leadY = targetPose.getY() - robotVelocity.vyMetersPerSecond * estimatedFlightTime;

        return new Pose2d(leadX, leadY, targetPose.getRotation());
    }
}
