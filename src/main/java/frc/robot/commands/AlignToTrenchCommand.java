package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.filter.SlewRateLimiter;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Drivetrain.SwerveDriveSubsystem;
import frc.robot.subsystems.Drivetrain.SwerveDriveConstants;
import org.littletonrobotics.junction.Logger;
import java.util.function.Supplier;

/**
 * Command to align and square the robot to the trenches.
 * 
 * Aligns robot Y position to nearest trench (Y = 0.64m or Y = 7.43m)
 * and rotates robot to nearest multiple of π (0° or 180°)
 * Allows manual X translation via controller stick input.
 */
public class AlignToTrenchCommand extends Command {
    
    // Trench Y coordinates in meters
    private static final double TRENCH_Y_BOTTOM = 0.64;
    private static final double TRENCH_Y_TOP = 7.43;
    
    // PID controllers for Y alignment and rotation
    private final PIDController yController;
    private final PIDController rotationController;
    
    private final SwerveDriveSubsystem drivetrain;
    private final Supplier<Double> xInputSupplier;
    private final SlewRateLimiter xLimiter;
    
    private double targetY;
    private double targetRotation;
    
    /**
     * Create an AlignToTrenchCommand.
     * 
     * @param drivetrain The swerve subsystem
     * @param xInputSupplier Supplier for X stick input (in range [-1, 1])
     * @param kP Proportional gain for both Y and rotation alignment
     * @param kI Integral gain for both Y and rotation alignment
     * @param kD Derivative gain for both Y and rotation alignment
     */
    public AlignToTrenchCommand(SwerveDriveSubsystem drivetrain, Supplier<Double> xInputSupplier, double kP, double kI, double kD) {
        this.drivetrain = drivetrain;
        this.xInputSupplier = xInputSupplier;
        this.yController = new PIDController(kP, kI, kD);
        this.rotationController = new PIDController(kP, kI, kD);
        this.xLimiter = new SlewRateLimiter(SwerveDriveConstants.DriveConstants.kTeleDriveMaxAccelerationMetersPerSecSq);
        
        // Set reasonable tolerances
        this.yController.setTolerance(0.05); // ±5cm
        this.rotationController.setTolerance(Math.toRadians(5)); // ±5°
        
        // Wrap rotation error to [-π, π]
        this.rotationController.enableContinuousInput(-Math.PI, Math.PI);
        
        addRequirements(drivetrain);
    }
    
    /**
     * Create an AlignToTrenchCommand with default PID gains.
     * 
     * @param drivetrain The swerve subsystem
     * @param xInputSupplier Supplier for X stick input (in range [-1, 1])
     */
    public AlignToTrenchCommand(SwerveDriveSubsystem drivetrain, Supplier<Double> xInputSupplier) {
        this(drivetrain, xInputSupplier, 3.0, 0.0, 0.0);
    }
    
    @Override
    public void initialize() {
        Pose2d currentPose = drivetrain.getPose();
        
        // Determine target Y: closest trench
        double distToBottom = Math.abs(currentPose.getY() - TRENCH_Y_BOTTOM);
        double distToTop = Math.abs(currentPose.getY() - TRENCH_Y_TOP);
        targetY = distToBottom < distToTop ? TRENCH_Y_BOTTOM : TRENCH_Y_TOP;
        
        // Determine target rotation: nearest multiple of π (0 or π)
        double currentRotation = currentPose.getRotation().getRadians();
        // Normalize to [-π, π]
        while (currentRotation > Math.PI) currentRotation -= 2 * Math.PI;
        while (currentRotation < -Math.PI) currentRotation += 2 * Math.PI;
        
        // Nearest multiple of π is either 0 or π
        targetRotation = Math.abs(currentRotation) < Math.PI / 2 ? 0 : Math.PI;
        
        Logger.recordOutput("AlignToTrench/TargetY", targetY);
        Logger.recordOutput("AlignToTrench/TargetRotation", Math.toDegrees(targetRotation));
    }
    
    @Override
    public void execute() {
        Pose2d currentPose = drivetrain.getPose();
        
        // Get X input from stick and apply deadband
        double xSpeed = -xInputSupplier.get();
        xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
        
        // Scale X input to max speed and apply acceleration limiting
        xSpeed = xSpeed * SwerveDriveConstants.DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        xSpeed = xLimiter.calculate(xSpeed);
        
        // Flip X speed for red alliance (like SwerveCommand does)
        if (drivetrain.shouldFlipPose()) {
            xSpeed = -xSpeed;
        }
        
        // Calculate Y velocity to align with target
        double yVelocity = yController.calculate(currentPose.getY(), targetY);
        
        // Calculate rotation velocity to align with target
        double rotationVelocity = rotationController.calculate(
            currentPose.getRotation().getRadians(), 
            targetRotation
        );
        
        // Clamp velocities to reasonable values
        yVelocity = Math.max(-2.0, Math.min(2.0, yVelocity)); // 2 m/s
        rotationVelocity = Math.max(-3.0, Math.min(3.0, rotationVelocity)); // 3 rad/s
        
        // Get the robot's rotation for field-relative conversion
        // Don't flip the pose - just use the actual robot rotation
        Rotation2d robotRotation = currentPose.getRotation();
        
        // Drive with field-relative velocity, allowing manual X control
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            xSpeed,
            yVelocity,
            rotationVelocity,
            robotRotation
        );
        
        drivetrain.driveVelocity(speeds);
        
        Logger.recordOutput("AlignToTrench/CurrentY", currentPose.getY());
        Logger.recordOutput("AlignToTrench/CurrentRotation", Math.toDegrees(currentPose.getRotation().getRadians()));
        Logger.recordOutput("AlignToTrench/YError", Math.abs(currentPose.getY() - targetY));
        Logger.recordOutput("AlignToTrench/RotationError", Math.abs(currentPose.getRotation().getRadians() - targetRotation));
        Logger.recordOutput("AlignToTrench/XInput", xSpeed);
    }
    
    @Override
    public void end(boolean interrupted) {
        // Stop the robot
        drivetrain.driveVelocity(new ChassisSpeeds());
    }
    
    @Override
    public boolean isFinished() {
        // Finished when both Y and rotation are within tolerance
        return false;
    }
}
