package frc.lib;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;

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
            cost += Math.abs(hood - currentAngleRad) * 2.0;
            cost += Math.abs(r.velocity - currentVelocity);
            cost += r.time * 0.3;

            if (cost < bestCost) {
                bestCost = cost;
                best = new ShotResult(yaw, hood, r.velocity, r.time);
            }
        }

        return best;
    }

    /**
     * @brief Solve launch velocity for a fixed hood angle.
     *
     * @param horizontalDist Horizontal distance to target (m)
     * @param dz             Vertical difference target - shooter (m)
     * @param angleRad       Hood angle (rad)
     *
     * @return ShotResult with velocity + time or null if impossible
     */
    private static ShotResult solveForAngle(double horizontalDist, double dz, double angleRad) {

        double loTime = 0.05; // lower time bound (s)
        double hiTime = 3.0; // upper time bound (s)

        for (int i = 0; i < 40; i++) {
            double t = (loTime + hiTime) / 2.0; // trial time

            // vertical/horizontal components required for this time
            double vy = (dz + 0.5 * G * t * t) / t; // vertical speed (m/s)
            double vx = horizontalDist / t; // horizontal speed (m/s)
            double impliedAngle = Math.atan2(vy, vx); // implied hood angle (rad)

            if (impliedAngle > angleRad)
                loTime = t;
            else
                hiTime = t;
        }

        double t = (loTime + hiTime) / 2.0;
        if (t <= 0)
            return null;

        double vy = (dz + 0.5 * G * t * t) / t;
        double vx = horizontalDist / t;
        double v = Math.hypot(vx, vy);

        if (v <= 0)
            return null;

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
}