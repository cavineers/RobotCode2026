package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import java.lang.Math;
import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import static frc.robot.subsystems.Drivetrain.SwerveDriveConstants.*;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Drivetrain.SwerveDriveSubsystem;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class SwerveCommand extends Command {

    private final SwerveDriveSubsystem swerveSubsystem;

    private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;

    public SwerveCommand(SwerveDriveSubsystem swerveSubsystem,
            Supplier<Double> xSpdFunction, 
            Supplier<Double> ySpdFunction, 
            Supplier<Double> turningSpdFunction){
        // Instance Variables
        this.swerveSubsystem = swerveSubsystem;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.turningSpdFunction = turningSpdFunction;
        this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationMetersPerSecSq);
        this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationMetersPerSecSq);
        this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationRadPerSecSq);
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {

        // Get real-time joystick inputs
        double xSpeed = -xSpdFunction.get();
        double ySpeed = -ySpdFunction.get();
        double turningSpeed = -turningSpdFunction.get();

        // Apply deadband -- compensated for when the joystick value does not return to exactly zero
        xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;
        turningSpeed = Math.abs(turningSpeed) > 0.1 ? turningSpeed : 0.0;

        // Scale inputs to max speed FIRST (convert from [-1,1] to velocity)
        xSpeed = xSpeed * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        ySpeed = ySpeed * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        turningSpeed = turningSpeed * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

        // THEN apply acceleration limiting (on actual velocities, not normalized inputs)
        xSpeed = xLimiter.calculate(xSpeed);
        ySpeed = yLimiter.calculate(ySpeed);
        turningSpeed = turningLimiter.calculate(turningSpeed);
     
        // Flipped
        boolean flipped = swerveSubsystem.shouldFlipPose();

        // Convert to field relative chassis speeds
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turningSpeed,
            flipped ? swerveSubsystem.getPose().getRotation().plus(Rotation2d.fromRadians(Math.PI)) : swerveSubsystem.getPose().getRotation());
        
        // Set the swerve modules to the specified speeds
        swerveSubsystem.driveVelocity(speeds);
        
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}