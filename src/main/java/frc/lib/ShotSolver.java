package frc.lib;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public final class ShotSolver {

    private static final double G = 9.81;

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

        for (double hood = minAngleRad; hood <= maxAngleRad; hood += Math.toRadians(0.25)) {

            ShotResult r = solveForAngle(horizontalDistance, dz, hood);
            if (r == null || !r.isValid())
                continue;
            if (r.velocity > maxVelocity)
                continue;

            double cost = 0.0;
            
            // Prefer lower, flatter trajectories (less affected by air resistance, faster)
            cost += r.time * 1; // penalize long flight time (MAY NEED TUNING)
            
            // Only penalize deviation from current state if we have a valid current state
            if (currentVelocity > 1.0) { // if shooter is already spinning
                cost += Math.abs(hood - currentAngleRad) * 2.0;
                cost += Math.abs(r.velocity - currentVelocity) * 0.5;
            }

            if (cost < bestCost) {
                bestCost = cost;
                best = new ShotResult(yaw, hood, r.velocity, r.time);
            }
        }

        return best;
    }

    /**
     * @brief Compute the best feasible shot accounting for robot velocity.
     *
     * <p>This overload adjusts for robot movement during time-of-flight by iteratively
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

        double aimLength = 5.0; // how far to visualize (meters)

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
     * <p>Computes where the projectile will land based on the shot parameters.
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
}