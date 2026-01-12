package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.ShotSolverSimplified;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Drivetrain.SwerveDriveConstants;
import frc.robot.subsystems.Drivetrain.SwerveDriveSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

/**
 * Simplified command that calculates angle to target and RPM from distance lookup.
 * Controls robot rotation to aim at target while allowing driver control of translation.
 */
public class ContinuousShotCalculationCommand extends Command {
    private final SwerveDriveSubsystem drivetrain;
    private final ShooterSubsystem shooter;
    private final Pose3d targetPose;
    private final Supplier<Double> xSpdFunction;
    private final Supplier<Double> ySpdFunction;
    
    // Tunable parameters
    private boolean useVelocityCompensation = true;
    private static final double kAimingSpeedMultiplier = 0.5; // Reduce speed to 50% while aiming for better control
    private static final double kRotationP = 3.0; // Proportional gain for rotation

    /**
     * Creates command with default BLUE HUB target.
     * Target height is 72 inches (1.8288 meters) above ground.
     */
    public ContinuousShotCalculationCommand(
            SwerveDriveSubsystem drivetrain, 
            ShooterSubsystem shooter,
            Supplier<Double> xSpdFunction,
            Supplier<Double> ySpdFunction) {
        this(drivetrain, shooter, new Pose3d(4.638, 4.075, 1.8288, new Rotation3d()), xSpdFunction, ySpdFunction);
    }

    /**
     * Creates command with custom target.
     */
    public ContinuousShotCalculationCommand(
            SwerveDriveSubsystem drivetrain, 
            ShooterSubsystem shooter, 
            Pose3d targetPose,
            Supplier<Double> xSpdFunction,
            Supplier<Double> ySpdFunction) {
        this.drivetrain = drivetrain;
        this.shooter = shooter;
        this.targetPose = targetPose;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        
        // Initialize SmartDashboard values
        SmartDashboard.putBoolean("Shooter/UseVelocityCompensation", useVelocityCompensation);
        
        addRequirements(drivetrain, shooter);
    }

    @Override
    public void initialize() {
        Logger.recordOutput("Shooter/CommandRunning", true);
    }

    @Override
    public void execute() {
        // Read tunable values from dashboard
        useVelocityCompensation = SmartDashboard.getBoolean("Shooter/UseVelocityCompensation", useVelocityCompensation);
        
        // Get driver translation inputs
        double xSpeed = -xSpdFunction.get();
        double ySpeed = -ySpdFunction.get();
        
        // Apply deadband
        xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;
        
        // Scale to max speed with aiming speed multiplier (NO rate limiting for constant velocity)
        double maxSpeed = SwerveDriveConstants.DriveConstants.kTeleDriveMaxSpeedMetersPerSecond * kAimingSpeedMultiplier;
        xSpeed = xSpeed * maxSpeed;
        ySpeed = ySpeed * maxSpeed;
        
        // Get current robot state
        Pose2d robotPose = drivetrain.getPose();
        ChassisSpeeds robotVelocity = drivetrain.getChassisSpeeds();
        
        // Convert 3D target to 2D for horizontal distance/angle calculations
        Pose2d targetPose2d = targetPose.toPose2d();
        
        // Calculate distance and initial RPM (for flight time calculation)
        double distance = ShotSolverSimplified.getDistanceToTarget(robotPose, targetPose2d);
        double calculatedRPM = ShotSolverSimplified.getRPMForDistance(distance);
        
        // Calculate flight time based on distance and projectile velocity
        // Convert RPM to linear velocity: v = (RPM * 2π * radius) / 60
        double projectileVelocity = (calculatedRPM * 2.0 * Math.PI * 0.05) / 60.0; // Using 0.05m as placeholder radius
        double flightTime = projectileVelocity > 0 ? distance / projectileVelocity : 0.5;
        
        // If using velocity compensation, recalculate for lead target
        Pose2d effectiveTargetPose2d = targetPose2d;
        if (useVelocityCompensation) {
            effectiveTargetPose2d = ShotSolverSimplified.getLeadTargetPose(
                robotVelocity, targetPose2d, flightTime);
            
            // Recalculate distance and RPM for the lead target
            distance = ShotSolverSimplified.getDistanceToTarget(robotPose, effectiveTargetPose2d);
            calculatedRPM = ShotSolverSimplified.getRPMForDistance(distance);
        }
        
        // Calculate angle to target
        Rotation2d angleToTarget = ShotSolverSimplified.getAngleToTarget(robotPose, effectiveTargetPose2d);

        // Calculate rotation speed to aim at target (simple P controller)
        double currentYaw = robotPose.getRotation().getRadians();
        double targetYaw = angleToTarget.getRadians();
        double yawError = targetYaw - currentYaw;
        
        // Normalize angle error to [-π, π]
        while (yawError > Math.PI) yawError -= 2 * Math.PI;
        while (yawError < -Math.PI) yawError += 2 * Math.PI;
        
        double rotationSpeed = kRotationP * yawError;
        
        // Clamp rotation speed to max
        double maxRotationSpeed = SwerveDriveConstants.DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;
        rotationSpeed = Math.max(-maxRotationSpeed, Math.min(maxRotationSpeed, rotationSpeed));
        
        // Check if robot is flipped
        boolean flipped = drivetrain.shouldFlipPose();
        
        // Convert to field relative chassis speeds and drive
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            xSpeed, 
            ySpeed, 
            rotationSpeed,
            flipped ? robotPose.getRotation().plus(Rotation2d.fromRadians(Math.PI)) : robotPose.getRotation()
        );
        drivetrain.driveVelocity(speeds);
        
        // Command shooter with calculated RPM
        shooter.setVelocity(calculatedRPM);
        
        // Calculate aim pose for visualization (1 meter in front of robot)
        double aimX = robotPose.getX() + Math.cos(angleToTarget.getRadians()) * 1.0;
        double aimY = robotPose.getY() + Math.sin(angleToTarget.getRadians()) * 1.0;
        Pose2d aimPose = new Pose2d(aimX, aimY, angleToTarget);
        
        // Create 3D pose for visualization
        Pose3d effectiveTarget = new Pose3d(
            effectiveTargetPose2d.getX(), 
            effectiveTargetPose2d.getY(), 
            targetPose.getZ(), 
            targetPose.getRotation()
        );
        
        // Log results
        Logger.recordOutput("Shooter/TargetPose", targetPose);
        Logger.recordOutput("Shooter/EffectiveTargetPose", effectiveTarget);
        Logger.recordOutput("Shooter/AimPose", aimPose);
        Logger.recordOutput("Shooter/AngleToTarget", angleToTarget.getDegrees());
        Logger.recordOutput("Shooter/CurrentYaw", Math.toDegrees(currentYaw));
        Logger.recordOutput("Shooter/YawError", Math.toDegrees(yawError));
        Logger.recordOutput("Shooter/Distance", distance);
        Logger.recordOutput("Shooter/CalculatedRPM", calculatedRPM);
        Logger.recordOutput("Shooter/CurrentRPM", shooter.getVelocityRPM());
        Logger.recordOutput("Shooter/FlightTime", flightTime);
        Logger.recordOutput("Shooter/UseVelocityCompensation", useVelocityCompensation);
        
        // Write back to SmartDashboard
        SmartDashboard.putBoolean("Shooter/UseVelocityCompensation", useVelocityCompensation);
        SmartDashboard.putNumber("Shooter/FlightTime", flightTime);
        SmartDashboard.putNumber("Shooter/CalculatedRPM", calculatedRPM);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        Logger.recordOutput("Shooter/CommandRunning", false);
        shooter.stop();
        drivetrain.driveVelocity(new ChassisSpeeds(0, 0, 0)); // Stop the robot
    }
}
