package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.FuelSim;
import frc.lib.ShotSolver;
import frc.lib.ShotSolver.ShotSolution;
import frc.robot.subsystems.Drivetrain.SwerveDriveSubsystem;
import frc.robot.subsystems.Shooter.ShooterConstants;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.Turret.Turret;
import org.littletonrobotics.junction.Logger;

/**
 * Continuously solves for and commands RPM, pitch, and turret heading for shooting on the fly
 */
public class AutoShootCommand extends Command {

    private final SwerveDriveSubsystem drivetrain;
    private final ShooterSubsystem shooter;
    private final Turret turret;
    /** May be null when running on a real robot or when FuelSim is not desired. */
    private final FuelSim fuelSim;

    private ShotSolution lastSolution = null;
    private final Timer shotTimer = new Timer();

    /**
     * Convenience constructor — no FuelSim (real robot or testing without sim).
     */
    public AutoShootCommand(SwerveDriveSubsystem drivetrain, ShooterSubsystem shooter, Turret turret) {
        this(drivetrain, shooter, turret, null);
    }

    /**
     * @param drivetrain drivetrain for pose / chassis-speeds (not required — read only)
     * @param shooter    shooter subsystem (required)
     * @param turret     turret subsystem (required)
     * @param fuelSim    particle simulation to call {@code launchFuel} on, or {@code null}
     */
    public AutoShootCommand(SwerveDriveSubsystem drivetrain, ShooterSubsystem shooter,
            Turret turret, FuelSim fuelSim) {
        this.drivetrain = drivetrain;
        this.shooter = shooter;
        this.turret = turret;
        this.fuelSim = fuelSim;
        addRequirements(shooter, turret);
    }

    @Override
    public void initialize() {
        turret.enableClosedLoop(true);
        shotTimer.restart();
        Logger.recordOutput("AutoShoot/Active", true);
    }

    @Override
    public void execute() {
        ShotSolution solution = ShotSolver.solveDynamic(
                drivetrain.getPose(),
                drivetrain.getFieldRelativeChassisSpeeds());

        lastSolution = solution;

        if (solution.isValid()) {
            shooter.setVelocity(solution.rpm());
            turret.setFieldRelativeTarget(solution.turretFieldAngleRad());
        } else {
            // Outside characterised range — hold turret in place
            turret.holdCurrentPosition();
        }

        Logger.recordOutput("AutoShoot/SolutionValid", solution.isValid());
        Logger.recordOutput("AutoShoot/ReadyToFire", isReadyToFire());

        if (solution.isValid()) {
            Translation3d shooterPos = ShotSolver.getShooterPosition(drivetrain.getPose());
            // ShooterPose3d: where the ball leaves, yaw = turret heading
            Pose3d shooterPose3d = new Pose3d(shooterPos,
                    new Rotation3d(0.0, 0.0, solution.turretFieldAngleRad()));
            Logger.recordOutput("AutoShoot/ShooterPose3d", shooterPose3d);

            // AimPointPose3d: the virtual target the turret is pointed at
            // Z matches the goal height so it renders correctly in AdvantageScope
            Translation2d aim = solution.aimPoint();
            Pose3d aimPose3d = new Pose3d(
                    new Translation3d(aim.getX(), aim.getY(), ShotSolver.GOAL_HEIGHT_METERS),
                    new Rotation3d());
            Logger.recordOutput("AutoShoot/AimPointPose3d", aimPose3d);
        } else {
            Logger.recordOutput("AutoShoot/ShooterPose3d", new Pose3d());
            Logger.recordOutput("AutoShoot/AimPointPose3d", new Pose3d());
        }

        // Fire a sim ball every 1 second regardless of ready state
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
     * True when the solver has a valid solution AND both the shooter and turret
     * have settled onto their targets. Use this to gate ball release.
     */
    public boolean isReadyToFire() {
        return lastSolution != null
                && lastSolution.isValid()
                && shooter.isAtTarget()
                && turret.isAtTarget();
    }
}
