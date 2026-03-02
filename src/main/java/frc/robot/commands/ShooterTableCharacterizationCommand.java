package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.FuelSim;
import frc.lib.ShotSolver;
import frc.robot.subsystems.Drivetrain.SwerveDriveSubsystem;
import frc.robot.subsystems.Shooter.ShooterConstants;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.Turret.Turret;
import org.littletonrobotics.junction.Logger;

public class ShooterTableCharacterizationCommand extends Command {

    private final SwerveDriveSubsystem drivetrain;
    private final ShooterSubsystem shooter;
    private final Turret turret;
    private final FuelSim fuelSim;

    private boolean firePending = false;

    /**
     * @param drivetrain pose source (not required — read only)
     * @param shooter    shooter subsystem (required)
     * @param turret     turret subsystem (required)
     * @param fuelSim    particle simulation, or {@code null} for real-robot use
     */
    public ShooterTableCharacterizationCommand(
            SwerveDriveSubsystem drivetrain,
            ShooterSubsystem shooter,
            Turret turret,
            FuelSim fuelSim) {
        this.drivetrain = drivetrain;
        this.shooter = shooter;
        this.turret = turret;
        this.fuelSim = fuelSim;
        addRequirements(shooter, turret);
    }

    @Override
    public void initialize() {
        firePending = false;
        turret.enableClosedLoop(true);
        Logger.recordOutput("ShooterCharacterization/Active", true);
    }

    @Override
    public void execute() {
        double rpm = shooter.getTunableRPM();
        double pitchDeg = shooter.getTunablePitchDegrees();

        // Spin up flywheel
        shooter.setVelocity(rpm);

        // Aim turret straight at the goal (simple, no SOTF) using current pose
        var simpleSolution = ShotSolver.solveSimple(drivetrain.getPose());
        turret.setFieldRelativeTarget(simpleSolution.turretFieldAngleRad());

        // Compute 3D distance so we can log it as the key for the LUT entry
        Translation3d shooterPos = ShotSolver.getShooterPosition(drivetrain.getPose());
        double distanceMeters = simpleSolution.effectiveDistanceMeters();

        Logger.recordOutput("ShooterCharacterization/DistanceMeters", distanceMeters);
        Logger.recordOutput("ShooterCharacterization/TunableRPM", rpm);
        Logger.recordOutput("ShooterCharacterization/TunablePitchDeg", pitchDeg);
        Logger.recordOutput("ShooterCharacterization/ShooterAtTarget", shooter.isAtTarget());
        Logger.recordOutput("ShooterCharacterization/TurretAtTarget", turret.isAtTarget());

        // Fire when requested (set by fire())
        if (firePending && shooter.isAtTarget() && turret.isAtTarget()) {
            firePending = false;
            logDataPoint(distanceMeters, rpm, pitchDeg);
            if (fuelSim != null) {
                launchSimFuel(rpm, pitchDeg, simpleSolution.turretFieldAngleRad(), shooterPos);
            }
        }
    }

    /**
     * Request a shot. Call this from a button binding (e.g.,
     * {@code secondaryController.a().onTrue(Commands.runOnce(charCmd::fire))}).
     * The shot fires on the next execute() loop where both shooter and turret
     * are at their targets, so there's no race condition.
     */
    public void fire() {
        firePending = true;
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
        turret.holdCurrentPosition();
        firePending = false;
        Logger.recordOutput("ShooterCharacterization/Active", false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    // -----------------------------------------------------------------------

    /**
     * Logs the confirmed data point. Copy these values straight into
     * {@link ShotSolver}'s static initialiser block:
     * {@code addEntry(distanceMeters, rpm, pitchDeg, <measured TOF>);}
     */
    private void logDataPoint(double distanceMeters, double rpm, double pitchDeg) {
        Logger.recordOutput("ShooterCharacterization/LastShot/DistanceMeters", distanceMeters);
        Logger.recordOutput("ShooterCharacterization/LastShot/RPM", rpm);
        Logger.recordOutput("ShooterCharacterization/LastShot/PitchDeg", pitchDeg);
        Logger.recordOutput("ShooterCharacterization/LastShot/LUTEntry",
                String.format("addEntry(%.2f, %.0f, %.1f, <tof>);", distanceMeters, rpm, pitchDeg));
    }

    /** Converts tunable values to FuelSim units and fires a particle from the shooter world position. */
    private void launchSimFuel(double rpm, double pitchDeg, double turretFieldAngleRad,
            Translation3d shooterPos) {
        double launchSpeedMps = rpm * Math.PI * ShooterConstants.kFlywheelRadiusMeters / 30.0;

        double pitchRad = Math.toRadians(pitchDeg);
        double hVel = Math.cos(pitchRad) * launchSpeedMps;
        double vVel = Math.sin(pitchRad) * launchSpeedMps;
        double xVel = hVel * Math.cos(turretFieldAngleRad);
        double yVel = hVel * Math.sin(turretFieldAngleRad);

        // Inherit robot momentum
        edu.wpi.first.math.kinematics.ChassisSpeeds fieldSpeeds =
                drivetrain.getFieldRelativeChassisSpeeds();
        xVel += fieldSpeeds.vxMetersPerSecond;
        yVel += fieldSpeeds.vyMetersPerSecond;

        fuelSim.spawnFuel(shooterPos, new Translation3d(xVel, yVel, vVel));
    }
}
