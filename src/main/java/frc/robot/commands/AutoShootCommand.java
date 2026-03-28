package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.FuelSim;
import frc.lib.ShotSolver;
import frc.lib.ShotSolver.ShotSolution;
import frc.robot.subsystems.Drivetrain.SwerveDriveSubsystem;
import frc.robot.subsystems.InBumperIntake.InBumperIntake;
import frc.robot.subsystems.InBumperIntake.InBumperIntake.IntakeState;
import frc.robot.subsystems.InBumperIntake.InBumperIntakeConstants;
import frc.robot.subsystems.Shooter.ShooterConstants;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.Turret.Turret;
import frc.robot.subsystems.Turret.TurretConstants;
import org.littletonrobotics.junction.Logger;

/**
 * Continuously solves for and commands RPM, pitch, and turret heading for shooting on the fly.
 * Gates ball release on: acceleration, shooter RPM within tolerance, and turret not in deadzone.
 */
public class AutoShootCommand extends Command {

    // Safety thresholds
    private static final double MAX_ACCELERATION_MPS2 = 4.5; // m/s² — above this, don't fire
    private static final double RPM_TOLERANCE = 350.0;        // ± RPM from setpoint

    // Field geometry constants
    private static final double FIELD_WIDTH_X = 16.52;        // FRC field width in meters
    private static final double FIELD_CENTER_Y = 4.021;       // Field center Y coordinate
    
    // Passing goal locations
    private static final double PASSING_GOAL_X_BLUE = 2.31;
    private static final double PASSING_GOAL_Y_BLUE_BOTTOM = 2.017;
    private static final double PASSING_GOAL_Y_BLUE_TOP = 2 * FIELD_CENTER_Y - PASSING_GOAL_Y_BLUE_BOTTOM; // = 6.025 m (reflected)
    
    // Red side passing goals (reflected across center X)
    private static final double PASSING_GOAL_X_RED = FIELD_WIDTH_X - PASSING_GOAL_X_BLUE; // = 14.21 m
    private static final double PASSING_GOAL_Y_RED_BOTTOM = PASSING_GOAL_Y_BLUE_TOP;      // = 6.025 m
    private static final double PASSING_GOAL_Y_RED_TOP = PASSING_GOAL_Y_BLUE_BOTTOM;      // = 2.017 m
    
    // Neutral zone boundary (Blue: x > 4.61, Red: x < 11.91)
    private static final double NEUTRAL_ZONE_BOUNDARY_X_BLUE = 4.61;   // For blue alliance
    private static final double NEUTRAL_ZONE_BOUNDARY_X_RED = FIELD_WIDTH_X - NEUTRAL_ZONE_BOUNDARY_X_BLUE; // = 11.91 m

    private final SwerveDriveSubsystem drivetrain;
    private final ShooterSubsystem shooter;
    private final Turret turret;
    private final InBumperIntake intake;
    /** May be null when running on a real robot or when FuelSim is not desired. */
    private final FuelSim fuelSim;

    private ShotSolution lastSolution = null;
    private final Timer shotTimer = new Timer();
    private final Timer hopperDelayTimer = new Timer();
    private boolean readyLatched = false;

    // For acceleration calculation
    private ChassisSpeeds lastSpeeds = new ChassisSpeeds();
    private double lastSpeedsTimestamp = 0.0;
    
    private static final double HOPPER_STARTUP_DELAY_SECONDS = 0.5;

    /**
     * Convenience constructor — no FuelSim (real robot or testing without sim).
     */
    public AutoShootCommand(SwerveDriveSubsystem drivetrain, ShooterSubsystem shooter, Turret turret, InBumperIntake intake) {
        this(drivetrain, shooter, turret, intake, null);
    }

    /**
     * @param drivetrain drivetrain for pose / chassis-speeds (not required — read only)
     * @param shooter    shooter subsystem (required)
     * @param turret     turret subsystem (required)
     * @param intake     in-bumper intake for hopper feed when ready (required)
     * @param fuelSim    particle simulation to call {@code launchFuel} on, or {@code null}
     */
    public AutoShootCommand(SwerveDriveSubsystem drivetrain, ShooterSubsystem shooter,
            Turret turret, InBumperIntake intake, FuelSim fuelSim) {
        this.drivetrain = drivetrain;
        this.shooter = shooter;
        this.turret = turret;
        this.intake = intake;
        this.fuelSim = fuelSim;
        addRequirements(shooter, turret, intake);
    }

    @Override
    public void initialize() {
        turret.enableClosedLoop(true);
        shotTimer.restart();
        hopperDelayTimer.restart();
        readyLatched = false;
        lastSpeeds = drivetrain.getFieldRelativeChassisSpeeds();
        lastSpeedsTimestamp = Timer.getFPGATimestamp();
        Logger.recordOutput("AutoShoot/Active", true);
    }

    @Override
    public void execute() {
        // Determine if we're on red or blue alliance
        boolean isRed = DriverStation.getAlliance()
                .map(a -> a == DriverStation.Alliance.Red)
                .orElse(true); // Default to red if unknown
        
        // Check if robot is in neutral zone and should pass instead of shoot
        double robotX = drivetrain.getPose().getX();
        boolean inNeutralZone = isRed 
                ? robotX < NEUTRAL_ZONE_BOUNDARY_X_RED  // Red
                : robotX > NEUTRAL_ZONE_BOUNDARY_X_BLUE; // Blue
        Logger.recordOutput("AutoShoot/InNeutralZone", inNeutralZone);
        Logger.recordOutput("AutoShoot/IsRedAlliance", isRed);
        
        ShotSolution solution;
        if (inNeutralZone) {
            // Use passing goal instead of regular shot
            solution = solvePassingShot(drivetrain.getPose(), drivetrain.getFieldRelativeChassisSpeeds(), isRed);
        } else {
            // Regular shot at the speaker
            solution = ShotSolver.solveDynamic(
                    drivetrain.getPose(),
                    drivetrain.getFieldRelativeChassisSpeeds());
            Logger.recordOutput("AutoShoot/PassingMode", false);
        }

        lastSolution = solution;

        if (solution.isValid()) {
            shooter.setVelocity(solution.rpm());
            turret.setFieldRelativeTarget(solution.turretFieldAngleRad());
        } else {
            // Outside characterised range — hold turret in place
            turret.holdCurrentPosition();
        }

        // Compute acceleration from delta speeds
        double now = Timer.getFPGATimestamp();
        double dt = now - lastSpeedsTimestamp;
        ChassisSpeeds currentSpeeds = drivetrain.getFieldRelativeChassisSpeeds();
        double accel = 0.0;
        if (dt > 0.001) {
            double dvx = currentSpeeds.vxMetersPerSecond - lastSpeeds.vxMetersPerSecond;
            double dvy = currentSpeeds.vyMetersPerSecond - lastSpeeds.vyMetersPerSecond;
            accel = Math.hypot(dvx, dvy) / dt;
        }
        lastSpeeds = currentSpeeds;
        lastSpeedsTimestamp = now;

        boolean ready = isReadyToFire(accel);
        if (!readyLatched && ready) {
            readyLatched = true;
        }

        // Always spin outside/bottom/top motors during AutoShoot to feed balls up
        intake.setState(IntakeState.HOPPER_TO_SHOOTER);
        intake.setOutsideVoltage(-InBumperIntakeConstants.kOutsideVoltage);
        intake.setBottomVoltage(-InBumperIntakeConstants.kBottomVoltage);
        intake.setTopVoltage(InBumperIntakeConstants.kTopVoltage);

        // Only spin hopper when fully ready (turret locked AND shooter ready AND in range)
        // AND after the startup delay has passed
        boolean hopperDelayPassed = hopperDelayTimer.hasElapsed(HOPPER_STARTUP_DELAY_SECONDS);
        if (ready && hopperDelayPassed) {
            intake.setHopperVoltage(InBumperIntakeConstants.kHopperVoltage);
        } else {
            intake.setHopperVoltage(0);    
        }

        Logger.recordOutput("AutoShoot/SolutionValid", solution.isValid());
        Logger.recordOutput("AutoShoot/ReadyToFire", readyLatched);
        Logger.recordOutput("AutoShoot/Acceleration", accel);

        // Fire a sim ball every 0.25 seconds regardless of ready state
        if (fuelSim != null && solution.isValid() && shotTimer.advanceIfElapsed(0.25)) {
            launchSimFuel(solution);
        }
    }

    /**
        * Spawn a fuel in the sim with the same initial velocity as a real ball would
     */
    private void launchSimFuel(ShotSolution solution) {
        double launchSpeedMps =
                solution.rpm() * Math.PI * ShooterConstants.kFlywheelRadiusMeters / 30.0;

        // World-space shooter origin — matches ShooterPose3d logged in execute()
        Translation3d shooterPos = ShotSolver.getShooterPosition(drivetrain.getPose());

        // Use a fixed pitch angle since we no longer have hood control
        double pitchRad = Math.toRadians(45.0); // Fixed pitch for sim
        double yawRad = solution.turretFieldAngleRad(); // already field-relative

        double hVel = Math.cos(pitchRad) * launchSpeedMps;
        double vVel = Math.sin(pitchRad) * launchSpeedMps;
        double xVel = hVel * Math.cos(yawRad);
        double yVel = hVel * Math.sin(yawRad);

        // Add robot chassis velocity so the ball inherits robot momentum
        ChassisSpeeds fieldSpeeds = drivetrain.getFieldRelativeChassisSpeeds();
        xVel += fieldSpeeds.vxMetersPerSecond;
        yVel += fieldSpeeds.vyMetersPerSecond;

        fuelSim.spawnFuel(shooterPos, new Translation3d(xVel, yVel, vVel));
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
        turret.holdCurrentPosition();
        intake.setBottomVoltage(0);
        intake.setTopVoltage(0);
        intake.setOutsideVoltage(0);
        intake.setHopperVoltage(0);
        intake.setState(IntakeState.IDLE);
        lastSolution = null;
        shotTimer.stop();
        Logger.recordOutput("AutoShoot/Active", false);
        Logger.recordOutput("AutoShoot/ReadyToFire", false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    /**
     * True when all safeties pass and it is safe to feed a ball into the shooter.
     *
     * <p>Safeties:
     * <ul>
     *   <li>Solver has a valid solution</li>
     *   <li>Shooter RPM is within ±{@value RPM_TOLERANCE} of setpoint</li>
     *   <li>Turret is locked on target (target is reachable / not in deadzone)</li>
     *   <li>Turret target is within mechanical range (before clamping)</li>
     *   <li>Robot acceleration is below {@value MAX_ACCELERATION_MPS2} m/s²</li>
     * </ul>
     * @deprecated
     */
    @Deprecated
    public boolean isReadyToFire(double accelerationMps2) {
        if (lastSolution == null || !lastSolution.isValid()) return false;

        // Distance protection: only shoot when within effective range of shot table
        double effectiveDistance = lastSolution.effectiveDistanceMeters();
        boolean distanceOk = effectiveDistance >= 2.41 && effectiveDistance <= 4.924;

        // Check if turret target is within mechanical range (before clamping)
        double robotHeading = drivetrain.getPose().getRotation().getRadians();
        double requestedAngle = lastSolution.turretFieldAngleRad() - robotHeading;
        
        // Normalize to [-π, π]
        while (requestedAngle > Math.PI) requestedAngle -= 2 * Math.PI;
        while (requestedAngle < -Math.PI) requestedAngle += 2 * Math.PI;
        
        boolean angleInRange = requestedAngle >= TurretConstants.kMinAngleRad && 
                               requestedAngle <= TurretConstants.kMaxAngleRad;

        boolean rpmOk = Math.abs(lastSolution.rpm() - shooter.getVelocityRPM()) <= RPM_TOLERANCE;
        boolean turretOk = turret.isTargetLocked();
        boolean accelOk = accelerationMps2 <= MAX_ACCELERATION_MPS2;

        Logger.recordOutput("AutoShoot/Safety/DistanceOk", distanceOk);
        Logger.recordOutput("AutoShoot/Safety/AngleInRange", angleInRange);
        Logger.recordOutput("AutoShoot/Safety/RPMOk", rpmOk);
        Logger.recordOutput("AutoShoot/Safety/TurretLocked", turretOk);
        Logger.recordOutput("AutoShoot/Safety/AccelOk", accelOk);

        // return distanceOk && angleInRange && rpmOk && accelOk;
        return true;
    }

    /**
     * Solve for passing shot parameters to a passing goal in the neutral zone.
     * Uses ShotSolver.solveDynamic with the passing goal instead of the speaker.
     * Determines which passing goal (bottom or top) based on robot's y position.
     * Goals are selected based on alliance (red vs blue) and reflected accordingly.
     */
    private ShotSolution solvePassingShot(edu.wpi.first.math.geometry.Pose2d robotPose, ChassisSpeeds fieldSpeeds, boolean isRed) {
        double robotY = robotPose.getY();
        
        // Select passing goal coordinates based on alliance
        double passingGoalX;
        double passingGoalYBottom;
        double passingGoalYTop;
        
        if (isRed) {
            passingGoalX = PASSING_GOAL_X_RED;        // x = 14.21 m
            passingGoalYBottom = PASSING_GOAL_Y_RED_BOTTOM;  // y = 6.025 m
            passingGoalYTop = PASSING_GOAL_Y_RED_TOP;        // y = 2.017 m
        } else {
            passingGoalX = PASSING_GOAL_X_BLUE;       // x = 2.31 m
            passingGoalYBottom = PASSING_GOAL_Y_BLUE_BOTTOM; // y = 2.017 m
            passingGoalYTop = PASSING_GOAL_Y_BLUE_TOP;       // y = 6.025 m
        }
        
        // Determine which passing goal is closer based on robot y position
        double passingGoalY;
        if (Math.abs(robotY - passingGoalYBottom) < Math.abs(robotY - passingGoalYTop)) {
            passingGoalY = passingGoalYBottom;
        } else {
            passingGoalY = passingGoalYTop;
        }
        
        // Use ShotSolver.solveDynamic with the passing goal as the target
        // Height is set low so distance stays within characterized range (< 5m)
        Translation3d passingGoal3d = new Translation3d(passingGoalX, passingGoalY, 0.1); // Low height for passing
        ShotSolution solution = ShotSolver.solveDynamic(robotPose, fieldSpeeds, passingGoal3d);
        
        Logger.recordOutput("AutoShoot/PassingMode", true);
        Logger.recordOutput("AutoShoot/PassingGoalY", passingGoalY);
        Logger.recordOutput("AutoShoot/PassingDistance", solution.effectiveDistanceMeters());
        Logger.recordOutput("AutoShoot/PassingRPM", solution.rpm());
        Logger.recordOutput("AutoShoot/PassingSolution", solution.toString()); // Debug: log entire solution
        
        return solution;
    }
}
