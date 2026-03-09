package frc.lib;

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
 * Shot solver for calculating shooter parameters based on robot position and velocity.
 * 
 * solveSimple: Uses lookup table with current position, no velocity compensation
 * solveDynamic: Calculates lookahead position based on time of flight, compensates for robot motion
 * 
 * All angles use WPILib field convention (CCW+, 0° = +X axis toward red alliance)
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
     * Shot parameters needed by shooter and turret subsystems.
     */
    public record ShotSolution(
            double rpm,
            double turretFieldAngleRad,
            double effectiveDistanceMeters,
            Translation2d aimPoint,
            boolean isValid) {

        @Override
        public String toString() {
            return String.format(
                    "ShotSolution{rpm=%.1f, turret=%.2f°, d*=%.3fm, aim=(%.2f,%.2f), valid=%b}",
                    rpm, Math.toDegrees(turretFieldAngleRad),
                    effectiveDistanceMeters, aimPoint.getX(), aimPoint.getY(), isValid);
        }
    }

    // -----------------------------------------------------------------------
    // Lookup tables - characterized with robot stationary
    // -----------------------------------------------------------------------

    private static final InterpolatingDoubleTreeMap RPM = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap TOF = new InterpolatingDoubleTreeMap();

    static {
        // distance (m), RPM, time of flight (s)
        addEntry(2.17, 1600, 0.9);
        addEntry(3.24, 1700, 0.9);
        addEntry(1.59, 1600, 1.0);
        addEntry(1.59, 1600, 1.0);
        addEntry(3.81, 1800, 1.0);
    }

    /**
     * Basic shot calculation - no compensation for robot movement.
     * Just looks up RPM and aims straight at the goal.
     */
    public static ShotSolution solveSimple(Pose2d robotPose) {
        Translation3d goal = getGoal();
        Translation3d shooter = getShooterPosition(robotPose);

        double dr = distanceTo(shooter, goal);
        double rpm = RPM.get(dr);

        Rotation2d turretAngle = goal.toTranslation2d().minus(shooter.toTranslation2d()).getAngle();
        ShotSolution solution = new ShotSolution(rpm, turretAngle.getRadians(), dr, goal.toTranslation2d(), isInRange(dr));

        logSolution("ShotSolver/Simple", solution, shooter.toTranslation2d(), goal.toTranslation2d());
        return solution;
    }

    /**
     * Shoot-on-the-fly with velocity compensation.
     * 
     * Works by calculating where the shooter will be when the fuel arrives at the goal.
     * Uses that "lookahead" position to look up shot parameters instead of current position.
     * This naturally compensates for robot velocity - if you're driving toward the goal,
     * the lookahead position is closer so you get lower RPM. Vice versa for driving away.
     */
    public static ShotSolution solveDynamic(Pose2d robotPose, ChassisSpeeds fieldSpeeds) {
        Translation3d goal = getGoal();
        Translation3d shooter = getShooterPosition(robotPose);

        double dr = distanceTo(shooter, goal);
        if (dr < 1e-6) {
            return solveSimple(robotPose);
        }

        double vx = fieldSpeeds.vxMetersPerSecond;
        double vy = fieldSpeeds.vyMetersPerSecond;

        // Iteratively converge on lookahead position
        // Initial guess uses TOF from current distance, then we refine it
        double tof = TOF.get(dr);
        Translation3d lookaheadShooter = shooter;
        double lookaheadDistance = dr;
        
        for (int i = 0; i < 20; i++) {
            // Calculate where shooter will be after TOF seconds
            double offsetX = vx * tof;
            double offsetY = vy * tof;

            // Lookahead shooter position is current position plus velocity * time of flight
            lookaheadShooter = new Translation3d(
                shooter.getX() + offsetX,
                shooter.getY() + offsetY,
                shooter.getZ()
            );
            
            lookaheadDistance = distanceTo(lookaheadShooter, goal);

            // Recalculate TOF from new position
            double newTof = TOF.get(lookaheadDistance);
            
            if (Math.abs(newTof - tof) < 0.001) {
                // Convergence achieved
                tof = newTof;
                break;
            }
            // Update TOF for next iteration
            tof = newTof;
        }

        double rpm = RPM.get(lookaheadDistance);

        // Turret aims from lookahead position, not current position
        Translation2d aimPoint = goal.toTranslation2d();
        Rotation2d turretAngle = aimPoint.minus(lookaheadShooter.toTranslation2d()).getAngle();

        ShotSolution solution = new ShotSolution(rpm, turretAngle.getRadians(), lookaheadDistance, aimPoint, isInRange(lookaheadDistance));

        logSolution("ShotSolver/Dynamic", solution, shooter.toTranslation2d(), aimPoint);
        Logger.recordOutput("ShotSolver/Dynamic/LookaheadDistance", lookaheadDistance);
        Logger.recordOutput("ShotSolver/Dynamic/ActualDistance", dr);
        Logger.recordOutput("ShotSolver/Dynamic/TimeOfFlight", tof);
        Logger.recordOutput("ShotSolver/Dynamic/RobotVelocity", new double[]{vx, vy});
        Logger.recordOutput("ShotSolver/Dynamic/LookaheadShooterPos", lookaheadShooter.toTranslation2d());
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

    // Rotates shooter offset from robot frame to field frame
    public static Translation3d getShooterPosition(Pose2d robotPose) {
        double heading = robotPose.getRotation().getRadians();
        double dx = SHOOTER_OFFSET.getX() * Math.cos(heading) - SHOOTER_OFFSET.getY() * Math.sin(heading);
        double dy = SHOOTER_OFFSET.getX() * Math.sin(heading) + SHOOTER_OFFSET.getY() * Math.cos(heading);
        return new Translation3d(
                robotPose.getX() + dx,
                robotPose.getY() + dy,
                SHOOTER_OFFSET.getZ());
    }

    private static double distanceTo(Translation3d launcher, Translation3d goal) {
        return launcher.getDistance(goal);
    }

    private static boolean isInRange(double d) {
        return d >= MIN_DISTANCE_METERS && d <= MAX_DISTANCE_METERS;
    }

    private static void logSolution(String prefix, ShotSolution s,
            Translation2d launcherPos, Translation2d aimPoint) {
        Logger.recordOutput(prefix + "/RPM", s.rpm());
        Logger.recordOutput(prefix + "/TurretFieldAngleDeg", Math.toDegrees(s.turretFieldAngleRad()));
        Logger.recordOutput(prefix + "/EffectiveDistanceMeters", s.effectiveDistanceMeters());
        Logger.recordOutput(prefix + "/IsValid", s.isValid());
        Logger.recordOutput(prefix + "/ShooterPosition", launcherPos);
        Logger.recordOutput(prefix + "/AimPoint", aimPoint);
    }

    private static void addEntry(double distM, double rpm, double tofSec) {
        RPM.put(distM, rpm);
        TOF.put(distM, tofSec);
    }
}
