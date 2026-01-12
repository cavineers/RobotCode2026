package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.ProjectilePhysics;
import frc.lib.ShotSolver;
import frc.lib.ShotSolver.ShotResult;
import frc.robot.subsystems.Drivetrain.SwerveDriveSubsystem;
import frc.robot.subsystems.Shooter.ShooterConstants;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import org.littletonrobotics.junction.Logger;

/**
 * Command that continuously calculates shot parameters using the ShotSolver.
 * This command runs in the background to provide real-time shot calculations
 * for visualization and debugging purposes.
 */
public class ContinuousShotCalculationCommand extends Command {
    private final SwerveDriveSubsystem drivetrain;
    private final ShooterSubsystem shooter;
    
    // Target configuration
    private final Pose2d targetPose;
    private final double shooterHeight;
    private final double targetHeight;
    private final double minHoodAngle;
    private final double maxHoodAngle;
    private final double initialHood;
    private final double maxShooterVelocity;
    
    // State tracking for smooth transitions
    private double lastCommandedHood;
    private double lastCommandedVelocity;
    private double lastYaw;
    
    // Smoothing constants (0.0 = no smoothing, 1.0 = instant change)
    private static final double kHoodSmoothing = 0.15;     // Slow hood transitions
    private static final double kVelocitySmoothing = 0.3;  // Moderate velocity transitions
    private static final double kYawSmoothing = 0.4;       // Faster yaw transitions
    
    // Enable/disable physics simulation (can be disabled for performance)
    private static final boolean kEnableProjectilePhysics = true;

    /**
     * Creates a new ContinuousShotCalculationCommand with default target (BLUE HUB).
     * 
     * @param drivetrain The swerve drive subsystem for robot pose and velocity
     * @param shooter The shooter subsystem for current flywheel velocity
     */
    public ContinuousShotCalculationCommand(SwerveDriveSubsystem drivetrain, ShooterSubsystem shooter) {
        this(
            drivetrain,
            shooter,
            new Pose2d(4.638, 4.075, new Rotation2d()), // BLUE HUB target
            0.56,  // shooter height (meters)
            1.83,  // target height (meters)
            Math.toRadians(15), // min hood angle
            Math.toRadians(75), // max hood angle
            Units.degreesToRadians(30.0), // current hood angle
            20.0   // max shooter velocity (m/s)
        );
    }

    /**
     * Creates a new ContinuousShotCalculationCommand with custom parameters.
     * 
     * @param drivetrain The swerve drive subsystem
     * @param shooter The shooter subsystem for current flywheel velocity
     * @param targetPose The target pose to shoot at
     * @param shooterHeight Height of shooter above ground (meters)
     * @param targetHeight Height of target above ground (meters)
     * @param minHoodAngle Minimum hood angle (radians)
     * @param maxHoodAngle Maximum hood angle (radians)
     * @param currentHood Current hood angle (radians)
     * @param maxShooterVelocity Maximum shooter velocity (m/s)
     */
    public ContinuousShotCalculationCommand(
            SwerveDriveSubsystem drivetrain,
            ShooterSubsystem shooter,
            Pose2d targetPose,
            double shooterHeight,
            double targetHeight,
            double minHoodAngle,
            double maxHoodAngle,
            double currentHood,
            double maxShooterVelocity) {
        this.drivetrain = drivetrain;
        this.shooter = shooter;
        this.targetPose = targetPose;
        this.shooterHeight = shooterHeight;
        this.targetHeight = targetHeight;
        this.minHoodAngle = minHoodAngle;
        this.maxHoodAngle = maxHoodAngle;
        this.initialHood = currentHood;
        this.maxShooterVelocity = maxShooterVelocity;
        
        // Initialize state tracking
        this.lastCommandedHood = currentHood;
        this.lastCommandedVelocity = 0.0;
        
        // Only require shooter - drivetrain is read-only
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        Logger.recordOutput("Shooter/ShotCalculationCommandRunning", true);
        System.out.println("ContinuousShotCalculationCommand started");
        
        // Reset state tracking
        lastCommandedHood = initialHood;
        lastCommandedVelocity = 0.0;
        lastYaw = 0.0;
    }

    @Override
    public void execute() {
        // Get current robot state
        Pose2d robotPose = drivetrain.getPose();
        ChassisSpeeds robotVelocity = drivetrain.getChassisSpeeds();
        
        // Get current shooter velocity from the shooter subsystem
        // Convert from RPM to m/s using the flywheel's tangential velocity
        // Formula: v (m/s) = (RPM * 2π * radius) / 60
        double currentShooterVelocityRPM = shooter.getVelocityRPM();
        double currentVelocity = rpmToMetersPerSecond(currentShooterVelocityRPM);

        // Calculate shot solution using last commanded state
        ShotResult shotResult = ShotSolver.solve(
            robotPose,
            robotVelocity,
            targetPose,
            shooterHeight,
            targetHeight,
            minHoodAngle,
            maxHoodAngle,
            lastCommandedHood,
            currentVelocity,
            maxShooterVelocity
        );

        // Apply exponential smoothing to prevent oscillation between similar states
        double smoothedYaw = lastYaw;
        double smoothedHood = lastCommandedHood;
        double smoothedVelocity = lastCommandedVelocity;
        
        if (shotResult != null && shotResult.isValid()) {
            // Exponential smoothing: new = alpha * target + (1 - alpha) * old
            smoothedYaw = kYawSmoothing * shotResult.yawRad + (1.0 - kYawSmoothing) * lastYaw;
            smoothedHood = kHoodSmoothing * shotResult.hoodRad + (1.0 - kHoodSmoothing) * lastCommandedHood;
            smoothedVelocity = kVelocitySmoothing * shotResult.velocity + (1.0 - kVelocitySmoothing) * lastCommandedVelocity;
            
            // Update state tracking
            lastYaw = smoothedYaw;
            lastCommandedHood = smoothedHood;
            lastCommandedVelocity = smoothedVelocity;
        }

        // Calculate aim pose (1 meter in front of robot in aim direction)
        Pose2d aimPose = robotPose;
        if (shotResult != null && shotResult.isValid()) {
            double aimX = robotPose.getX() + Math.cos(smoothedYaw) * 1.0;
            double aimY = robotPose.getY() + Math.sin(smoothedYaw) * 1.0;
            aimPose = new Pose2d(aimX, aimY, new Rotation2d(smoothedYaw));
        }

        // Command the shooter to the calculated velocity
        if (shotResult != null && shotResult.isValid()) {
            double targetRPM = metersPerSecondToRpm(smoothedVelocity);
            shooter.setVelocity(targetRPM);
        } else {
            // No valid solution, stop the shooter
            shooter.stop();
            lastCommandedVelocity = 0.0;
        }

        // Log all shot calculation results
        Logger.recordOutput("Shooter/AimPose", aimPose);
        Logger.recordOutput("Shooter/HoodAngleDeg", shotResult != null ? Units.radiansToDegrees(smoothedHood) : 0.0);
        Logger.recordOutput("Shooter/CalculatedVelocityMPS", shotResult != null ? shotResult.velocity : 0);
        Logger.recordOutput("Shooter/CalculatedVelocityRPM", shotResult != null ? metersPerSecondToRpm(shotResult.velocity) : 0);
        Logger.recordOutput("Shooter/CurrentVelocityMPS", currentVelocity);
        Logger.recordOutput("Shooter/CurrentVelocityRPM", currentShooterVelocityRPM);
        Logger.recordOutput("Shooter/ShotResultValid", shotResult != null && shotResult.isValid());
        Logger.recordOutput("Shooter/LandingPose",
            ShotSolver.getLandingPose(
                new Pose3d(robotPose.getX(), robotPose.getY(), shooterHeight, new Rotation3d()),
                shooterHeight,
                shotResult
            )
        );
        
        // Simulate and log accurate trajectory with air resistance (if enabled)
        if (kEnableProjectilePhysics && shotResult != null && shotResult.isValid()) {
            Pose3d shooterPose3d = new Pose3d(robotPose.getX(), robotPose.getY(), shooterHeight, new Rotation3d());
            
            // Create initial projectile state with robot velocity
            ProjectilePhysics.ProjectileState initialState = ProjectilePhysics.createInitialStateWithRobotVelocity(
                shooterPose3d,
                robotVelocity.vxMetersPerSecond,
                robotVelocity.vyMetersPerSecond,
                smoothedYaw,
                smoothedHood,
                smoothedVelocity
            );
            
            // Simulate full trajectory (0.02s time steps, 5s max time, 0m ground level)
            ProjectilePhysics.ProjectileState[] trajectory = ProjectilePhysics.simulateTrajectory(
                initialState,
                0.1,   // 100ms time steps (reduced from 20ms for better performance)
                5.0,   // 5 second max simulation
                0.0    // ground level
            );
            
            // Convert to Pose3d array for visualization
            Pose3d[] trajectoryPoses = new Pose3d[trajectory.length];
            for (int i = 0; i < trajectory.length; i++) {
                trajectoryPoses[i] = trajectory[i].toPose3d();
            }
            
            Logger.recordOutput("Shooter/SimulatedTrajectory", trajectoryPoses);
            
            // Log final landing position
            if (trajectory.length > 0) {
                ProjectilePhysics.ProjectileState landing = trajectory[trajectory.length - 1];
                Logger.recordOutput("Shooter/SimulatedLandingPose", landing.toPose3d());
                Logger.recordOutput("Shooter/FlightTime", landing.time);
            }
        }
    }

    @Override
    public boolean isFinished() {
        return false; // Runs continuously
    }

    @Override
    public void end(boolean interrupted) {
        Logger.recordOutput("Shooter/ShotCalculationCommandRunning", false);
        System.out.println("ContinuousShotCalculationCommand ended. Interrupted: " + interrupted);
        shooter.stop();
    }

    /**
     * Converts flywheel RPM to projectile exit velocity in m/s.
     * Assumes the projectile exits at the tangential velocity of the flywheel.
     * 
     * @param rpm Flywheel velocity in RPM
     * @return Exit velocity in m/s
     */
    private double rpmToMetersPerSecond(double rpm) {
        // v = (RPM * 2π * radius) / 60
        return (rpm * 2.0 * Math.PI * ShooterConstants.kFlywheelRadiusMeters) / 60.0;
    }

    /**
     * Converts projectile exit velocity in m/s to flywheel RPM.
     * Inverse of rpmToMetersPerSecond.
     * 
     * @param metersPerSecond Exit velocity in m/s
     * @return Flywheel velocity in RPM
     */
    private double metersPerSecondToRpm(double metersPerSecond) {
        // RPM = (v * 60) / (2π * radius)
        return (metersPerSecond * 60.0) / (2.0 * Math.PI * ShooterConstants.kFlywheelRadiusMeters);
    }
}
