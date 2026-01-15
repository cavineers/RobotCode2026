package frc.lib;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public final class ShotSolverSimplified {

    private static final double GRAVITY = 9.81; // m/s^2
    private static final double WHEEL_RADIUS = 0.0762; // meters (3 inches diameter = 1.5" radius) - TODO: Update with actual wheel diameter
    private static final int MAX_ITERATIONS = 5; // For iterative convergence

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
     * Complete shot solution including effective target and flight time.
     */
    public static class ShotSolution {
        public final ShotParameters shotParams;
        public final Pose2d effectiveTarget;
        public final double flightTime;
        public final Rotation2d yawAngle;
        public final boolean validSolution;

        public ShotSolution(ShotParameters shotParams, Pose2d effectiveTarget, double flightTime, Rotation2d yawAngle, boolean validSolution) {
            this.shotParams = shotParams;
            this.effectiveTarget = effectiveTarget;
            this.flightTime = flightTime;
            this.yawAngle = yawAngle;
            this.validSolution = validSolution;
        }
    }

    // Lookup tables: distance (meters) -> parameter
    private static final InterpolatingDoubleTreeMap distanceToRPM = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap distanceToPitch = new InterpolatingDoubleTreeMap();

    static {
        // TODO: Fill with real characterization data
        // These dummy values are more aggressive to reach a target ~1.8m higher
        distanceToRPM.put(1.0, 2000.0);
        distanceToRPM.put(2.0, 2500.0);
        distanceToRPM.put(3.0, 3000.0);
        distanceToRPM.put(4.0, 3500.0);
        distanceToRPM.put(5.0, 4000.0);
        
        // Steeper angles to reach higher targets
        distanceToPitch.put(1.0, 45.0);  
        distanceToPitch.put(2.0, 50.0);  
        distanceToPitch.put(3.0, 55.0);  
        distanceToPitch.put(4.0, 58.0);  
        distanceToPitch.put(5.0, 60.0);  
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

        // Calculate lead target position
        double leadX = targetPose.getX() - robotVelocity.vxMetersPerSecond * estimatedFlightTime;
        double leadY = targetPose.getY() - robotVelocity.vyMetersPerSecond * estimatedFlightTime;

        // Calculate angle to lead target
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
     * Calculate flight time using ballistic motion with vertical offset.
     * Solves: Δz = v_z*t - (1/2)*g*t²
     * Using quadratic formula: (1/2)*g*t² - v_z*t + Δz = 0
     * 
     * @param deltaZ Vertical offset (target height - shooter height) in meters
     * @param pitchDegrees Launch angle in degrees
     * @param rpm Flywheel RPM
     * @return Flight time in seconds, or -1 if no real solution exists
     */
    public static double calculateFlightTime(double deltaZ, double pitchDegrees, double rpm) {
        // Convert RPM to tangential velocity
        double velocity = (rpm * 2.0 * Math.PI * WHEEL_RADIUS) / 60.0;
        
        // Decompose into vertical component
        double pitchRad = Math.toRadians(pitchDegrees);
        double v_z = velocity * Math.sin(pitchRad);
        
        // Quadratic formula: a*t² + b*t + c = 0
        // (1/2)*g*t² - v_z*t + Δz = 0
        double a = 0.5 * GRAVITY;
        double b = -v_z;
        double c = deltaZ;
        
        double discriminant = b * b - 4 * a * c;
        
        if (discriminant < 0) {
            // No real solution - can't reach target
            System.out.println("[FlightTime] FAILED: Negative discriminant - cannot reach target! " +
                             "deltaZ=" + String.format("%.3f", deltaZ) + "m, " +
                             "pitch=" + String.format("%.1f", pitchDegrees) + "°, " +
                             "rpm=" + String.format("%.0f", rpm) + ", " +
                             "v_z=" + String.format("%.2f", v_z) + "m/s");
            return -1;
        }
        
        // Return the positive root (forward in time)
        double t1 = (-b + Math.sqrt(discriminant)) / (2 * a);
        double t2 = (-b - Math.sqrt(discriminant)) / (2 * a);
        
        // Use the smaller positive time (first arrival)
        if (t1 > 0 && t2 > 0) {
            return Math.min(t1, t2);
        } else if (t1 > 0) {
            return t1;
        } else if (t2 > 0) {
            return t2;
        } else {
            System.out.println("[FlightTime] FAILED: No positive time solutions! " +
                             "t1=" + String.format("%.3f", t1) + "s, " +
                             "t2=" + String.format("%.3f", t2) + "s");
            return -1; // No positive solution
        }
    }

    /**
     * Calculate horizontal distance the ball travels during flight.
     * 
     * @param pitchDegrees Launch angle in degrees
     * @param rpm Flywheel RPM
     * @param flightTime Time of flight in seconds
     * @return Horizontal distance in meters
     */
    public static double calculateHorizontalDistance(double pitchDegrees, double rpm, double flightTime) {
        // Convert RPM to tangential velocity
        double velocity = (rpm * 2.0 * Math.PI * WHEEL_RADIUS) / 60.0;
        
        // Decompose into horizontal component
        double pitchRad = Math.toRadians(pitchDegrees);
        double v_h = velocity * Math.cos(pitchRad);
        
        return v_h * flightTime;
    }

    /**
     * Calculate complete shot solution with velocity compensation.
     * Iteratively solves for effective target accounting for robot motion.
     * 
     * @param robotPose Current robot position (3D, includes shooter height)
     * @param robotVelocity Field-relative robot velocity
     * @param targetPose Target position (3D, includes target height)
     * @param useVelocityCompensation Whether to compensate for robot motion
     * @return Complete shot solution with parameters, effective target, and flight time
     */
    public static ShotSolution calculateShotSolution(
            Pose3d robotPose,
            ChassisSpeeds robotVelocity,
            Pose3d targetPose,
            boolean useVelocityCompensation) {
        
        // Step 1: Calculate initial field-plane vector to target
        double deltaX = targetPose.getX() - robotPose.getX();
        double deltaY = targetPose.getY() - robotPose.getY();
        double distance = Math.hypot(deltaX, deltaY);
        
        // Step 2: Calculate vertical offset
        double deltaZ = targetPose.getZ() - robotPose.getZ();
        
        // Initial guess: lookup parameters for straight-line distance
        ShotParameters shotParams = getShotParameters(distance);
        double flightTime = 0.5; // Initial guess
        boolean validSolution = true;
        
        // Step 3-7: Iterative solution with velocity compensation
        Pose2d effectiveTarget2d = targetPose.toPose2d();
        
        if (useVelocityCompensation) {
            for (int i = 0; i < MAX_ITERATIONS; i++) {
                // Calculate flight time from vertical motion
                flightTime = calculateFlightTime(deltaZ, shotParams.pitchDegrees, shotParams.rpm);
                if (flightTime < 0) {
                    // Can't reach target with current parameters - use fallback
                    System.out.println("[ShotSolution] FAILED: Cannot calculate effective target - using fallback");
                    flightTime = 0.5; // Fallback time
                    validSolution = false;
                    break;
                }
                
                // Calculate robot displacement during flight
                double robotDeltaX = robotVelocity.vxMetersPerSecond * flightTime;
                double robotDeltaY = robotVelocity.vyMetersPerSecond * flightTime;
                
                // Calculate effective target (where to aim)
                double effectiveX = targetPose.getX() - robotDeltaX;
                double effectiveY = targetPose.getY() - robotDeltaY;
                effectiveTarget2d = new Pose2d(effectiveX, effectiveY, targetPose.getRotation().toRotation2d());
                
                // Recalculate distance to effective target
                double newDistance = getDistanceToTarget(robotPose.toPose2d(), effectiveTarget2d);
                
                // Check convergence
                if (Math.abs(newDistance - distance) < 0.01) {
                    // Converged
                    break;
                }
                
                // Update parameters for new distance
                distance = newDistance;
                shotParams = getShotParameters(distance);
            }
        } else {
            // No velocity compensation - just calculate flight time
            flightTime = calculateFlightTime(deltaZ, shotParams.pitchDegrees, shotParams.rpm);
            if (flightTime < 0) {
                System.out.println("[ShotSolution] FAILED: Cannot reach target - using fallback");
                flightTime = 0.5; // Fallback
                validSolution = false;
            }
        }
        
        // Calculate final yaw angle to effective target
        Rotation2d yawAngle = getAngleToTarget(robotPose.toPose2d(), effectiveTarget2d);
        
        return new ShotSolution(shotParams, effectiveTarget2d, flightTime, yawAngle, validSolution);
    }

    /**
     * Get lead target position for visualization.
     * 
     * @param robotVelocity       Robot field-relative velocity
     * @param targetPose          Original target position
     * @param estimatedFlightTime Estimated time for projectile to reach target
     *                            (seconds)
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
