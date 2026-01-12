package frc.lib;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public final class ShotSolver {

    private static final double G = 9.81;

    // Empirical distance-to-RPM lookup table (populated with characterization data)
    private static final InterpolatingDoubleTreeMap distanceToRPM = new InterpolatingDoubleTreeMap();

    static {
        // TODO: Populate with real characterization data
        // Format: distanceToRPM.put(distanceMeters, motorRPM);
        // Example placeholder values:
        distanceToRPM.put(1.0, 1500.0);  // 1m -> 1500 RPM
        distanceToRPM.put(2.0, 2000.0);  // 2m -> 2000 RPM
        distanceToRPM.put(3.0, 2500.0);  // 3m -> 2500 RPM
        distanceToRPM.put(4.0, 3000.0);  // 4m -> 3000 RPM
        distanceToRPM.put(5.0, 3500.0);  // 5m -> 3500 RPM
    }

    public static class ShotResult {
        public final double yawRad;
        public final double hoodRad;
        public final double velocity;
        public final double time;

        public ShotResult(double yawRad, double hoodRad, double velocity, double time) {
            this.yawRad = yawRad;
            this.hoodRad = hoodRad;
            this.velocity = velocity;
            this.time = time;
        }

        public boolean isValid() {
            return !Double.isNaN(velocity) && velocity > 0;
        }
    }

    /**
     * @brief Compute the best feasible shot to hit a target.
     *
     * @param robotPose       Current robot pose on field (meters)
     * @param targetPose      Target pose on field (meters)
     * @param shooterHeight   Release height of ball (m)
     * @param targetHeight    Target height (m)
     * @param minAngleRad     Minimum hood angle (rad)
     * @param maxAngleRad     Maximum hood angle (rad)
     * @param currentAngleRad Current hood angle (rad)
     * @param currentVelocity Current shooter exit speed (m/s)
     * @param maxVelocity     Max allowed shooter speed (m/s)
     *
     * @return Best valid shot result or null if no solution
     */
    public static ShotResult solve(
            Pose2d robotPose,
            Pose2d targetPose,
            double shooterHeight,
            double targetHeight,
            double minAngleRad,
            double maxAngleRad,
            double currentAngleRad,
            double currentVelocity,
            double maxVelocity) {
        double dx = targetPose.getX() - robotPose.getX();
        double dy = targetPose.getY() - robotPose.getY();
        double dz = targetHeight - shooterHeight;

        double yaw = Math.atan2(dy, dx);
        double horizontalDistance = Math.hypot(dx, dy);

        ShotResult best = null;
        double bestCost = Double.POSITIVE_INFINITY;
        
        // Hysteresis threshold: new solution must be this much better to switch
        final double HYSTERESIS_THRESHOLD = 0.00; // 5% improvement required

        for (double hood = minAngleRad; hood <= maxAngleRad; hood += Math.toRadians(0.25)) {

            ShotResult r = solveForAngle(horizontalDistance, dz, hood);
            if (r == null || !r.isValid())
                continue;
            if (r.velocity > maxVelocity)
                continue;

            double cost = 0.0;
            
            // Prioritize minimizing drastic changes from current state
            // This keeps the shooter stable and reduces mechanical stress
            double angleDelta = Math.abs(hood - currentAngleRad);
            double velocityDelta = Math.abs(r.velocity - currentVelocity);
            
            // Heavy penalty for large changes
            cost += angleDelta * 5.0;      // Penalize hood angle changes
            cost += velocityDelta * 2.0;   // Penalize velocity changes
            
            // Prefer higher, more lofted trajectories (like basketball shots)
            // Penalize low angles - reward high angles
            double minLoftAngle = Math.toRadians(35); // Prefer angles above 35 degrees
            if (hood < minLoftAngle) {
                cost += (minLoftAngle - hood) * 10.0; // Heavy penalty for flat shots
            }
            
            // Calculate and reward higher vertex (peak height)
            // Vertex height = initial_z + (v_z^2) / (2*g)
            double vz = r.velocity * Math.sin(hood);
            double peakHeight = shooterHeight + (vz * vz) / (2.0 * G);
            double desiredMinPeak = targetHeight + 1.0; // Want peak at least 1m above target
            if (peakHeight < desiredMinPeak) {
                cost += (desiredMinPeak - peakHeight) * 3.0; // Penalty for low vertex
            }
            
            // Small penalty for flight time (prefer faster shots when all else equal)
            cost += r.time * 0.1;

            // Apply hysteresis: require new solution to be significantly better
            double costThreshold = bestCost * (1.0 - HYSTERESIS_THRESHOLD);
            if (cost < costThreshold) {
                bestCost = cost;
                best = new ShotResult(yaw, hood, r.velocity, r.time);
            }
        }

        return best;
    }

    /**
     * @brief Compute the best feasible shot accounting for robot velocity.
     *
     * This overload adjusts for robot movement during time-of-flight by iteratively
     * solving for the lead position where the robot will be when the projectile arrives.
     *
     * @param robotPose       Current robot pose on field (meters)
     * @param robotVelocity   Current robot velocity (field-relative, m/s)
     * @param targetPose      Target pose on field (meters)
     * @param shooterHeight   Release height of ball (m)
     * @param targetHeight    Target height (m)
     * @param minAngleRad     Minimum hood angle (rad)
     * @param maxAngleRad     Maximum hood angle (rad)
     * @param currentAngleRad Current hood angle (rad)
     * @param currentVelocity Current shooter exit speed (m/s)
     * @param maxVelocity     Max allowed shooter speed (m/s)
     *
     * @return Best valid shot result accounting for robot motion, or null if no solution
     */
    public static ShotResult solve(
            Pose2d robotPose,
            ChassisSpeeds robotVelocity,
            Pose2d targetPose,
            double shooterHeight,
            double targetHeight,
            double minAngleRad,
            double maxAngleRad,
            double currentAngleRad,
            double currentVelocity,
            double maxVelocity) {

        // Iteratively solve for lead position
        Pose2d adjustedRobotPose = robotPose;
        ShotResult lastResult = null;

        for (int iter = 0; iter < 5; iter++) {
            // Solve from adjusted position
            ShotResult result = solve(
                    adjustedRobotPose,
                    targetPose,
                    shooterHeight,
                    targetHeight,
                    minAngleRad,
                    maxAngleRad,
                    currentAngleRad,
                    currentVelocity,
                    maxVelocity);

            if (result == null || !result.isValid()) {
                return lastResult; // return last valid result or null
            }

            lastResult = result;

            // Adjust robot pose forward by velocity * time-of-flight
            double dt = result.time;
            double dx = robotVelocity.vxMetersPerSecond * dt;
            double dy = robotVelocity.vyMetersPerSecond * dt;

            adjustedRobotPose = new Pose2d(
                    robotPose.getX() + dx,
                    robotPose.getY() + dy,
                    robotPose.getRotation());

            // Check convergence
            if (Math.hypot(dx, dy) < 0.01) {
                break; // converged to ~1cm
            }
        }

        return lastResult;
    }

    /**
     * @brief Solve launch velocity for a fixed hood angle using proper ballistics.
     *
     * @param horizontalDist Horizontal distance to target (m)
     * @param dz             Vertical difference target - shooter (m)
     * @param angleRad       Hood angle (rad)
     *
     * @return ShotResult with velocity + time or null if impossible
     */
    private static ShotResult solveForAngle(double horizontalDist, double dz, double angleRad) {
        // Projectile motion: y = x*tan(θ) - (g*x²)/(2*v²*cos²(θ))
        // Rearranging for v: v² = (g*x²) / (2*cos²(θ)*(x*tan(θ) - y))
        // where x = horizontalDist, y = dz, θ = angleRad
        
        double cosAngle = Math.cos(angleRad);
        double tanAngle = Math.tan(angleRad);
        
        // Denominator: (x*tan(θ) - y)
        double denom = horizontalDist * tanAngle - dz;
        
        // Must be positive (shooting upward enough to clear the height difference)
        if (denom <= 0) {
            return null;
        }
        
        // Solve for velocity squared
        double vSquared = (G * horizontalDist * horizontalDist) / (2.0 * cosAngle * cosAngle * denom);
        
        if (vSquared <= 0) {
            return null;
        }
        
        double v = Math.sqrt(vSquared);
        
        // Calculate time of flight: t = x / (v * cos(θ))
        double t = horizontalDist / (v * cosAngle);
        
        if (t <= 0) {
            return null;
        }
        
        // yaw is set by the caller (computed from robot/target pose), so set to 0 here
        return new ShotResult(0, angleRad, v, t);
    }

    /**
     * @brief Build a Pose3d showing where the shooter is aiming
     *
     * @param robotPose current robot pose
     * @param shot      result from ShotSolver
     * @return Pose3d representing aim point in 3D
     * @note This is used for visualization only
     */
    public static Pose3d getAimingPose(Pose3d robotPose, ShotSolver.ShotResult shot) {

        double aimLength = 0.0; // how far to visualize (meters)

        double dx = Math.cos(shot.hoodRad) * Math.cos(shot.yawRad);
        double dy = Math.cos(shot.hoodRad) * Math.sin(shot.yawRad);
        double dz = Math.sin(shot.hoodRad);

        double x = robotPose.getX() + dx * aimLength;
        double y = robotPose.getY() + dy * aimLength;
        double z = robotPose.getZ() + dz * aimLength;

        return new Pose3d(
                x,
                y,
                z,
                new Rotation3d(
                        shot.hoodRad, // pitch
                        shot.yawRad, // yaw
                        0));
    }

    /**
     * @brief Calculate the landing position of a shot (assuming no robot velocity).
     *
     * Computes where the projectile will land based on the shot parameters.
     * Useful for visualization in AdvantageScope.
     *
     * @param robotPose     Current robot pose (meters)
     * @param shooterHeight Height of shooter release point (m)
     * @param shot          Shot result from solver
     *
     * @return Pose3d of where projectile lands, or null if shot is invalid
     */
    public static Pose3d getLandingPose(Pose3d robotPose, double shooterHeight, ShotResult shot) {
        if (shot == null || !shot.isValid()) {
            return null;
        }

        // Calculate horizontal distance traveled
        double horizontalDist = shot.velocity * Math.cos(shot.hoodRad) * shot.time;

        // Calculate landing position
        double landingX = robotPose.getX() + horizontalDist * Math.cos(shot.yawRad);
        double landingY = robotPose.getY() + horizontalDist * Math.sin(shot.yawRad);

        // Calculate final height (should be target height if solver worked correctly)
        double initialVy = shot.velocity * Math.sin(shot.hoodRad);
        double landingZ = shooterHeight + initialVy * shot.time - 0.5 * G * shot.time * shot.time;

        return new Pose3d(
                landingX,
                landingY,
                landingZ,
                new Rotation3d(0, 0, shot.yawRad)); // facing direction of shot
    }

    /**
     * @brief Simple interpolation-based solver using empirical distance-to-RPM data.
     *
     * Uses a lookup table populated from characterization shots. Hood angle is
     * assumed constant. This is a failsafe when the ballistic solver fails or for
     * quick tuning during practice.
     *
     * @param robotPose  Current robot pose (meters)
     * @param targetPose Target pose (meters)
     * @param hoodRad    Fixed hood angle (rad)
     *
     * @return Motor RPM required for the shot, or 0 if distance is out of range
     */
    public static double solveInterpolated(Pose2d robotPose, Pose2d targetPose, double hoodRad) {
        double dx = targetPose.getX() - robotPose.getX();
        double dy = targetPose.getY() - robotPose.getY();
        double distance = Math.hypot(dx, dy);

        // Interpolate RPM from the lookup table
        return distanceToRPM.get(distance);
    }

    /**
     * @brief Calculate the angle the robot needs to face to aim at the target.
     *
     * Returns the field-relative angle (yaw) from robot to target center.
     * Use this to align the robot before shooting.
     *
     * @param robotPose  Current robot pose (meters)
     * @param targetPose Target pose (meters)
     *
     * @return Field-relative angle to target in radians
     */
    public static double getAngleToTarget(Pose2d robotPose, Pose2d targetPose) {
        double dx = targetPose.getX() - robotPose.getX();
        double dy = targetPose.getY() - robotPose.getY();
        return Math.atan2(dy, dx);
    }

    /**
     * @brief Calculate the angle the robot needs to face to aim at the target (Rotation2d).
     *
     * @param robotPose  Current robot pose (meters)
     * @param targetPose Target pose (meters)
     *
     * @return Field-relative Rotation2d to target
     */
    public static Rotation2d getRotationToTarget(Pose2d robotPose, Pose2d targetPose) {
        return new Rotation2d(getAngleToTarget(robotPose, targetPose));
    }
}
