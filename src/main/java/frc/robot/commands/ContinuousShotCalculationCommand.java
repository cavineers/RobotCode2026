package frc.robot.commands;

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
    
    /**
     * Shooting solution modes.
     */
    public enum ShootingMode {
        SIMPLE_LOOKUP,              // Mode 0: Lookup table, no velocity compensation
        LOOKUP_WITH_VELOCITY        // Mode 1: Lookup table with simple velocity compensation
    }
    
    private final SwerveDriveSubsystem drivetrain;
    private final ShooterSubsystem shooter;
    private final Pose3d targetPose;
    private final Supplier<Double> xSpdFunction;
    private final Supplier<Double> ySpdFunction;
    
    // Tunable parameters
    private ShootingMode shootingMode = ShootingMode.LOOKUP_WITH_VELOCITY;
    private static final double kAimingSpeedMultiplier = 0.5; // Reduce speed to 50% while aiming for better control
    private static final double kRotationP = 5.0; // Proportional gain for rotation

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
        SmartDashboard.putNumber("Shooter/ShootingMode", shootingMode.ordinal());
        SmartDashboard.putString("Shooter/ShootingModeName", shootingMode.name());
        
        addRequirements(drivetrain, shooter);
    }

    @Override
    public void initialize() {
        Logger.recordOutput("Shooter/CommandRunning", true);
    }

    @Override
    public void execute() {
        // Read shooting mode from dashboard (0=Simple, 1=Lookup+Velocity)
        int modeIndex = (int) SmartDashboard.getNumber("Shooter/ShootingMode", shootingMode.ordinal());
        if (modeIndex >= 0 && modeIndex < ShootingMode.values().length) {
            shootingMode = ShootingMode.values()[modeIndex];
        }
        
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
        ChassisSpeeds robotVelocity = drivetrain.getFieldRelativeChassisSpeeds(); // MUST be field-relative!
        
        // Convert 3D target to 2D for horizontal distance/angle calculations
        Pose2d targetPose2d = targetPose.toPose2d();
        
        // Calculate shooting solution based on selected mode
        double distance;
        double calculatedRPM;
        double calculatedPitch;
        double flightTime;
        double horizontalVelocity;
        Pose2d effectiveTargetPose2d;
        Rotation2d angleToTarget;
        
        switch (shootingMode) {
            case SIMPLE_LOOKUP:
                // Mode 0: Simple lookup table, no velocity compensation
                distance = ShotSolverSimplified.getDistanceToTarget(robotPose, targetPose2d);
                ShotSolverSimplified.ShotParameters simpleParams = ShotSolverSimplified.getShotParameters(distance);
                
                calculatedRPM = simpleParams.rpm;
                calculatedPitch = simpleParams.pitchDegrees;
                effectiveTargetPose2d = targetPose2d;
                angleToTarget = ShotSolverSimplified.getAngleToTarget(robotPose, targetPose2d);
                
                // Calculate flight time for logging only (not used for aiming)
                double simpleVelocity = (simpleParams.rpm * 2.0 * Math.PI * 0.05) / 60.0;
                horizontalVelocity = simpleVelocity * Math.cos(Math.toRadians(simpleParams.pitchDegrees));
                flightTime = horizontalVelocity > 0 ? distance / horizontalVelocity : 0.5;
                break;
                
            case LOOKUP_WITH_VELOCITY:
            default:
                // Mode 1: Lookup table with simple velocity compensation (original method)
                distance = ShotSolverSimplified.getDistanceToTarget(robotPose, targetPose2d);
                ShotSolverSimplified.ShotParameters velocityParams = ShotSolverSimplified.getShotParameters(distance);
                
                // Calculate flight time based on horizontal projectile velocity
                double projectileVelocity = (velocityParams.rpm * 2.0 * Math.PI * 0.05) / 60.0;
                horizontalVelocity = projectileVelocity * Math.cos(Math.toRadians(velocityParams.pitchDegrees));
                flightTime = horizontalVelocity > 0 ? distance / horizontalVelocity : 0.5;
                
                // Calculate lead target position
                effectiveTargetPose2d = ShotSolverSimplified.getLeadTargetPose(
                    robotVelocity, targetPose2d, flightTime);
                
                // Recalculate distance and shot parameters for the lead target
                distance = ShotSolverSimplified.getDistanceToTarget(robotPose, effectiveTargetPose2d);
                velocityParams = ShotSolverSimplified.getShotParameters(distance);
                
                // Recalculate flight time for the new distance (iterative improvement)
                projectileVelocity = (velocityParams.rpm * 2.0 * Math.PI * 0.05) / 60.0;
                horizontalVelocity = projectileVelocity * Math.cos(Math.toRadians(velocityParams.pitchDegrees));
                flightTime = horizontalVelocity > 0 ? distance / horizontalVelocity : 0.5;
                
                calculatedRPM = velocityParams.rpm;
                calculatedPitch = velocityParams.pitchDegrees;
                angleToTarget = ShotSolverSimplified.getAngleToTarget(robotPose, effectiveTargetPose2d);
                break;
        }

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
        Logger.recordOutput("Shooter/CalculatedPitch", calculatedPitch);
        Logger.recordOutput("Shooter/CurrentRPM", shooter.getVelocityRPM());
        Logger.recordOutput("Shooter/FlightTime", flightTime);
        Logger.recordOutput("Shooter/HorizontalVelocity", horizontalVelocity);
        Logger.recordOutput("Shooter/ShootingMode", shootingMode.ordinal());
        Logger.recordOutput("Shooter/ShootingModeName", shootingMode.name());
        
        // Write back to SmartDashboard
        SmartDashboard.putNumber("Shooter/ShootingMode", shootingMode.ordinal());
        SmartDashboard.putString("Shooter/ShootingModeName", shootingMode.name());
        SmartDashboard.putNumber("Shooter/FlightTime", flightTime);
        SmartDashboard.putNumber("Shooter/CalculatedRPM", calculatedRPM);
        SmartDashboard.putNumber("Shooter/CalculatedPitch", calculatedPitch);
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