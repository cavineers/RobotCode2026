package frc.lib;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import org.littletonrobotics.junction.Logger;

/**
 * Figures out what RPM, hood angle, and turret heading to use for a given robot
 * state.
 *
 * Two modes:
 * solveSimple — straight lookup table, no velocity compensation.
 * solveDynamic — Newton's method SOTF. Finds a virtual distance d* that
 * accounts for
 * the robot's momentum being added to the ball.
 *
 * All angles are field-relative, WPILib convention (CCW positive, 0 = +X).
 */
public class ShotSolver {

    private static final Translation3d GOAL_BLUE = new Translation3d(Units.inchesToMeters(181.56),
            Units.inchesToMeters(158.32), Units.inchesToMeters(72));
    private static final Translation3d GOAL_RED = new Translation3d(Units.inchesToMeters(468.56),
            Units.inchesToMeters(158.32), Units.inchesToMeters(72));

    // Shooter position in the robot frame (forward, lateral, height).
    // TODO: update with real measurement
    private static final Translation3d SHOOTER_OFFSET = new Translation3d(0.381, 0.0, 0.5);

    public static final double SHOOTER_HEIGHT_METERS = SHOOTER_OFFSET.getZ();

    // -----------------------------------------------------------------------
    // Valid shot range — solutions outside this are flagged isValid=false
    // -----------------------------------------------------------------------

    public static final double MIN_DISTANCE_METERS = 0.9;
    public static final double MAX_DISTANCE_METERS = 5.5;
    // Goal opening height — shared by both goals, used for 3D logging
    public static final double GOAL_HEIGHT_METERS = Units.inchesToMeters(72);

    /**
     * Everything the shooter and turret need to take the shot.
     *
     * @param rpm                     flywheel speed
     * @param pitchDegrees            hood angle, degrees up from horizontal
     * @param turretFieldAngleRad     field-relative turret heading to command
     * @param effectiveDistanceMeters the d* the solver converged on
     * @param aimPoint                field-relative 2D point the turret is aimed at
     *                                (goal for simple, laterally-shifted for dynamic)
     * @param isValid                 false if d* is outside the characterised range
     */
    public record ShotSolution(
            double rpm,
            double pitchDegrees,
            double turretFieldAngleRad,
            double effectiveDistanceMeters,
            Translation2d aimPoint,
            boolean isValid) {

        @Override
        public String toString() {
            return String.format(
                    "ShotSolution{rpm=%.1f, pitch=%.2f°, turret=%.2f°, d*=%.3fm, aim=(%.2f,%.2f), valid=%b}",
                    rpm, pitchDegrees, Math.toDegrees(turretFieldAngleRad),
                    effectiveDistanceMeters, aimPoint.getX(), aimPoint.getY(), isValid);
        }
    }

    // -----------------------------------------------------------------------
    // Lookup tables
    // Key: distance from launcher to goal opening (meters).
    // -----------------------------------------------------------------------

    // Shared by both solvers — solveSimple just doesn't use TOF
    private static final InterpolatingDoubleTreeMap RPM = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap PITCH = new InterpolatingDoubleTreeMap();
    // TOF = horizontal travel time (s), used by solveDynamic for Vel(d) = d/Tof(d)
    private static final InterpolatingDoubleTreeMap TOF = new InterpolatingDoubleTreeMap();

    static {
        // dist (m) RPM pitch (°) tof (s)
        addEntry(2.17, 1600, 70.0, 0.9);
        addEntry(3.24, 1700, 60.0, 0.9);
        addEntry(1.59, 1600, 85.0, 1.0);
        addEntry(1.59, 1600, 85.0, 1.0);
        addEntry(3.81, 1800, 60.0, 1.0);

    }

    // -----------------------------------------------------------------------
    // Newton's method settings
    // -----------------------------------------------------------------------

    private static final double EPSILON = 0.001; // central-difference step (m) for numerical derivative of Vel(d)
    private static final double CONVERGENCE_THRESHOLD = 0.005; // stop when |f(d)| < this (m/s)
    private static final int MAX_ITERATIONS = 10;

    /**
     * Simple lookup — no velocity compensation.
     * Computes 3D distance from the launcher to the goal, reads RPM/pitch from the
     * table,
     * and points the turret straight at the goal.
     */
    public static ShotSolution solveSimple(Pose2d robotPose) {
        Translation3d goal = getGoal();
        Translation3d shooter = getShooterPosition(robotPose);

        double dr = distanceTo(shooter, goal);
        double rpm = RPM.get(dr);
        double pitch = PITCH.get(dr);

        Rotation2d turretAngle = goal.toTranslation2d().minus(shooter.toTranslation2d()).getAngle();
        ShotSolution solution = new ShotSolution(rpm, pitch, turretAngle.getRadians(), dr, goal.toTranslation2d(), isInRange(dr));

        logSolution("ShotSolver/Simple", solution, shooter.toTranslation2d(), goal.toTranslation2d());
        return solution;
    }

    /**
     * Newton's method shoot-on-the-fly.
     *
     * The robot's velocity adds to the ball's velocity at release. To compensate:
     * - The radial component (toward/away from goal) is cancelled by finding a d*
     * whose
     * LUT horizontal velocity equals v_s - v_radial. That d* drives RPM and pitch.
     * - The lateral component (perpendicular to the goal ray) drifts the ball
     * sideways during flight. The turret aim point is shifted opposite to that drift by
     * v_lateral * tof.
     */
    public static ShotSolution solveDynamic(Pose2d robotPose, ChassisSpeeds fieldSpeeds) {
        Translation3d goal = getGoal();
        Translation3d shooter = getShooterPosition(robotPose);
        Translation2d toGoalXY = goal.toTranslation2d().minus(shooter.toTranslation2d());

        double dr = distanceTo(shooter, goal);
        if (dr < 1e-6) {
            return solveSimple(robotPose);
        }

        // Decompose robot velocity into radial (toward goal) and lateral
        // (perpendicular) components
        double ux = toGoalXY.getX() / toGoalXY.getNorm(); // the "u" denotes unit vector toward the goal (magnitude 1)
        double uy = toGoalXY.getY() / toGoalXY.getNorm();
        double lx = -uy;  // rotate u 90° CCW to get the "lateral" unit vector pointing left of the goal
        double ly = ux;

        double vx = fieldSpeeds.vxMetersPerSecond;
        double vy = fieldSpeeds.vyMetersPerSecond;

        // Dot product of velocity with unit vectors gives us the radial and lateral components 
        // Simply said: vrRadial is how much of the robot's velocity is helping or hurting the shot, and vrLateral is how much it's pushing the ball sideways
        double vrRadial = vx * ux + vy * uy;
        double vrLateral = vx * lx + vy * ly; 

        // Find d* such that Vel(d*) = Vel(dr) - vrRadial
        double vc = vel(dr) - vrRadial;
        double d = dr;
        for (int i = 0; i < MAX_ITERATIONS; i++) {
            double f = vel(d) - vc; // how far off are we from the target velocity compensation? We want this to be zero, which would mean "the LUT-prescribed shot velocity at distance d exactly cancels out the robot's radial velocity, giving us the correct net ball velocity to hit the goal"
            if (Math.abs(f) < CONVERGENCE_THRESHOLD)
                break;

            double deriv = (vel(d + EPSILON) - vel(d - EPSILON)) / (2.0 * EPSILON);
            if (Math.abs(deriv) < 1e-9)
                break;

            d -= f / deriv; // Newton's method update: move d in the direction that would reduce f to zero. The step size is f/deriv, which is how much we expect f to change if we change d by a little bit. So this is like saying "if I increase d by this much, f would go down to zero, so let's do that".
            d = Math.max(d, 0.0);
        }

        double rpm = RPM.get(d);
        double pitch = PITCH.get(d);
        double tof = TOF.get(d);

        // Shift the aim point opposite to where the ball will drift laterally
        double lateralOffset = vrLateral * tof;
        Translation2d aimPoint = goal.toTranslation2d()
                .minus(new Translation2d(lx * lateralOffset, ly * lateralOffset));
        Rotation2d turretAngle = aimPoint.minus(shooter.toTranslation2d()).getAngle();

        ShotSolution solution = new ShotSolution(rpm, pitch, turretAngle.getRadians(), d, aimPoint, isInRange(dr));

        logSolution("ShotSolver/Dynamic", solution, shooter.toTranslation2d(), aimPoint);
        Logger.recordOutput("ShotSolver/Dynamic/RadialVelocity", vrRadial);
        Logger.recordOutput("ShotSolver/Dynamic/LateralVelocity", vrLateral);
        Logger.recordOutput("ShotSolver/Dynamic/LateralOffset", lateralOffset);
        Logger.recordOutput("ShotSolver/Dynamic/TimeOfFlight", tof);
        return solution;
    }

    // -----------------------------------------------------------------------
    // Helpers
    // -----------------------------------------------------------------------

    private static Translation3d getGoal() {
        boolean isRed = DriverStation.getAlliance()
                .map(a -> a == DriverStation.Alliance.Red)
                .orElse(false);
        return isRed ? GOAL_RED : GOAL_BLUE;
    }

    // Rotates the robot-frame shooter offset into the field frame and returns the
    // full 3D position.
    // Z is preserved directly since the robot drives on flat ground.
    public static Translation3d getShooterPosition(Pose2d robotPose) {
        double heading = robotPose.getRotation().getRadians();
        double dx = SHOOTER_OFFSET.getX() * Math.cos(heading) - SHOOTER_OFFSET.getY() * Math.sin(heading);
        double dy = SHOOTER_OFFSET.getX() * Math.sin(heading) + SHOOTER_OFFSET.getY() * Math.cos(heading);
        return new Translation3d(
                robotPose.getX() + dx,
                robotPose.getY() + dy,
                SHOOTER_OFFSET.getZ());
    }

    // Straight-line distance between the launcher exit and the goal opening
    private static double distanceTo(Translation3d launcher, Translation3d goal) {
        return launcher.getDistance(goal);
    }

    // Vel(d) = d / Tof(d) — horizontal shot velocity the LUT prescribes for
    // distance d
    private static double vel(double d) {
        double tof = TOF.get(MathUtil.clamp(d, MIN_DISTANCE_METERS, MAX_DISTANCE_METERS));
        return (tof < 1e-9) ? 0.0 : d / tof;
    }

    private static boolean isInRange(double d) {
        return d >= MIN_DISTANCE_METERS && d <= MAX_DISTANCE_METERS;
    }

    private static void logSolution(String prefix, ShotSolution s,
            Translation2d launcherPos, Translation2d aimPoint) {
        Logger.recordOutput(prefix + "/RPM", s.rpm());
        Logger.recordOutput(prefix + "/PitchDegrees", s.pitchDegrees());
        Logger.recordOutput(prefix + "/TurretFieldAngleDeg", Math.toDegrees(s.turretFieldAngleRad()));
        Logger.recordOutput(prefix + "/EffectiveDistanceMeters", s.effectiveDistanceMeters());
        Logger.recordOutput(prefix + "/IsValid", s.isValid());
        Logger.recordOutput(prefix + "/ShooterPosition", launcherPos);
        Logger.recordOutput(prefix + "/AimPoint", aimPoint);
    }

    private static void addEntry(double distM, double rpm, double pitchDeg, double tofSec) {
        RPM.put(distM, rpm);
        PITCH.put(distM, pitchDeg);
        TOF.put(distM, tofSec);
    }
}
