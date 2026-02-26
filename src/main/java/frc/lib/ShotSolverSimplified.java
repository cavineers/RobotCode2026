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

    /**
     * Extended shot parameters that include time of flight and compensated distance.
     * Used by Newton's method to return all necessary information for accurate SOTF.
     */
    public static class ShotParametersWithTOF {
        public final double rpm;
        public final double pitchDegrees;
        public final double timeOfFlight;
        public final double compensatedDistance;

        public ShotParametersWithTOF(double rpm, double pitchDegrees, double tof, double distance) {
            this.rpm = rpm;
            this.pitchDegrees = pitchDegrees;
            this.timeOfFlight = tof;
            this.compensatedDistance = distance;
        }
    }

    // Lookup tables: distance (meters) -> parameter
    private static final InterpolatingDoubleTreeMap distanceToRPM = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap distanceToPitch = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap distanceToTOF = new InterpolatingDoubleTreeMap();

    // Newton's method parameters
    private static final double EPSILON = 0.001; // Small value for derivative approximation
    private static final int MAX_ITERATIONS = 10; // Max iterations for Newton's method
    private static final double CONVERGENCE_THRESHOLD = 0.005; // Velocity convergence threshold (m/s)

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
        
        // Time of flight (seconds) for each distance
        // TODO: Characterize these values - they depend on shooter RPM, pitch, and projectile ballistics
        distanceToTOF.put(1.0, 0.3);
        distanceToTOF.put(2.0, 0.5);
        distanceToTOF.put(3.0, 0.7);
        distanceToTOF.put(4.0, 0.9);
        distanceToTOF.put(5.0, 1.1);
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
     * Get lead target position
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

    /**
     * Get time of flight for a given distance using linear interpolation.
     * 
     * @param distanceMeters Distance to target in meters
     * @return Time of flight in seconds
     */
    public static double getTOFForDistance(double distanceMeters) {
        return distanceToTOF.get(distanceMeters);
    }

    /**
     * Calculate horizontal velocity for a given distance.
     * Vel(d) = d / TOF(d)
     * 
     * @param distanceMeters Distance to target in meters
     * @return Required horizontal velocity in m/s
     */
    private static double getVelocityForDistance(double distanceMeters) {
        double tof = distanceToTOF.get(distanceMeters);
        return distanceMeters / tof;
    }

    /**
     * Approximate the derivative of velocity with respect to distance.
     * Uses central difference method: Vel'(d) ≈ (Vel(d+ε) - Vel(d-ε)) / (2ε)
     * 
     * @param distanceMeters Distance to evaluate derivative at
     * @return Approximate derivative of velocity
     */
    private static double getVelocityDerivative(double distanceMeters) {
        double lowVel = getVelocityForDistance(distanceMeters - EPSILON);
        double highVel = getVelocityForDistance(distanceMeters + EPSILON);
        return (highVel - lowVel) / (2.0 * EPSILON);
    }

    /**
     * Calculate shot parameters compensated for robot velocity using Newton's method.
     * 
     * This method compensates for radial robot velocity (toward/away from target) by:
     * 1. Calculating the required ball velocity after accounting for robot momentum
     * 2. Using Newton's method to find what distance's parameters produce that velocity
     * 3. Calculating the actual time of flight for those parameters over the actual distance
     * 
     * Uses Newton's method to solve: f(d) = Vel(d) - v_c = 0
     * Iteration: d_(n+1) = d_n - f(d_n) / f'(d_n)
     * 
     * @param robotPose     Current robot position
     * @param robotVelocity Robot field-relative velocity
     * @param targetPose    Target position
     * @return ShotParametersWithTOF containing RPM, pitch, actual TOF, and compensated distance
     */
    public static ShotParametersWithTOF getShotParametersWithNewton(
            Pose2d robotPose,
            ChassisSpeeds robotVelocity,
            Pose2d targetPose) {

        // Calculate raw distance from robot to target
        double robotDistance = getDistanceToTarget(robotPose, targetPose);
        
        // Get baseline shot velocity from lookup table
        double shotVelocity = getVelocityForDistance(robotDistance);
        
        // Calculate robot velocity component in direction of target
        double dx = targetPose.getX() - robotPose.getX();
        double dy = targetPose.getY() - robotPose.getY();
        double angleToTarget = Math.atan2(dy, dx);
        
        // Project robot velocity onto shot direction
        double robotVelX = robotVelocity.vxMetersPerSecond;
        double robotVelY = robotVelocity.vyMetersPerSecond;
        double robotVelMagnitude = Math.hypot(robotVelX, robotVelY);
        double robotVelAngle = Math.atan2(robotVelY, robotVelX);
        double robotVelInShotDirection = robotVelMagnitude * Math.cos(robotVelAngle - angleToTarget);
        
        // Calculate compensated velocity: v_c = v_s - v_r
        // This is the velocity the ball needs to leave the shooter at
        double compensatedVelocity = shotVelocity - robotVelInShotDirection;
        
        // Use Newton's method to find distance d* such that Vel(d*) = compensatedVelocity
        double currentDistance = robotDistance; // Start with robot's actual distance
        double currentVelocity = getVelocityForDistance(currentDistance);
        
        for (int i = 0; i < MAX_ITERATIONS && Math.abs(currentVelocity - compensatedVelocity) > CONVERGENCE_THRESHOLD; i++) {
            double velocityError = currentVelocity - compensatedVelocity;
            double velocityDerivative = getVelocityDerivative(currentDistance);
            
            // Newton's method update: d_(n+1) = d_n - f(d_n) / f'(d_n)
            currentDistance -= velocityError / velocityDerivative;
            
            // Update velocity for next iteration
            currentVelocity = getVelocityForDistance(currentDistance);
        }
        
        // Get shot parameters from the compensated distance
        ShotParameters params = getShotParameters(currentDistance);
        
        // Calculate ACTUAL time of flight:
        // Time for a ball shot with these compensated params to travel the actual distance
        // (NOT the TOF from lookup table, which is for different distance/params combination)
        double ballVelocity = (params.rpm * 2.0 * Math.PI * 0.05) / 60.0; // Convert RPM to m/s (assuming 0.05m flywheel radius)
        double horizontalVelocity = ballVelocity * Math.cos(Math.toRadians(params.pitchDegrees));
        double actualTOF = horizontalVelocity > 0 ? robotDistance / horizontalVelocity : 0.5;
        
        return new ShotParametersWithTOF(params.rpm, params.pitchDegrees, actualTOF, currentDistance);
    }
}