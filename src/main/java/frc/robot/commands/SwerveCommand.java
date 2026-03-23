package frc.robot.commands;

import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Drivetrain.SwerveDriveConstants.DriveConstants;
import frc.robot.subsystems.Climber.ClimberConstants;
import frc.robot.subsystems.Drivetrain.SwerveDriveConstants;
import frc.robot.subsystems.Drivetrain.SwerveDriveSubsystem;
import edu.wpi.first.math.controller.PIDController;

public class SwerveCommand extends Command {

    private final SwerveDriveSubsystem swerveSubsystem;

    private final Supplier<Boolean> alignTrench;
    private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
    private final Supplier<Double> speedMultiplier;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
    private double trenchP = 5.0;
    private double angle;
    private double desiredAngle;
    private final LoggedNetworkNumber tuningTrenchP = new LoggedNetworkNumber("/Tuning/Drivetrain/kTrenchP", trenchP);
    PIDController trenchPID = new PIDController(tuningTrenchP.get(),0,0);

    public SwerveCommand(SwerveDriveSubsystem swerveSubsystem,
            Supplier<Boolean> alignTrench,
            Supplier<Double> xSpdFunction, 
            Supplier<Double> ySpdFunction, 
            Supplier<Double> turningSpdFunction){
        this(swerveSubsystem, alignTrench, xSpdFunction, ySpdFunction, turningSpdFunction, () -> 1.0);
    }

    public SwerveCommand(SwerveDriveSubsystem swerveSubsystem,
            Supplier<Boolean> alignTrench,
            Supplier<Double> xSpdFunction, 
            Supplier<Double> ySpdFunction, 
            Supplier<Double> turningSpdFunction,
            Supplier<Double> speedMultiplier){
        // Instance Variables
        this.swerveSubsystem = swerveSubsystem;
        this.alignTrench = alignTrench;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.turningSpdFunction = turningSpdFunction;
        this.speedMultiplier = speedMultiplier;
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
        double scalar = speedMultiplier.get();
        xSpeed = xSpeed * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond * scalar;
        ySpeed = ySpeed * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond * scalar;
        turningSpeed = turningSpeed * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond * scalar;

        // THEN apply acceleration limiting (on actual velocities, not normalized inputs)
        xSpeed = xLimiter.calculate(xSpeed);
        ySpeed = yLimiter.calculate(ySpeed);
        turningSpeed = turningLimiter.calculate(turningSpeed);

        if (alignTrench.get() && angle >= Rotation2d.kCW_Pi_2.getRadians()){
            turningSpeed = this.getTrenchTurningVelocity();
            desiredAngle = Rotation2d.kPi.getRadians();
        }
        else if (alignTrench.get() && angle <= Rotation2d.kCCW_Pi_2.getRadians()){
            turningSpeed = this.getTrenchTurningVelocity();
            desiredAngle = Rotation2d.kZero.getRadians();
        }
     
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
    
    public double getTrenchTurningVelocity() {
        angle = swerveSubsystem.getRotation().getRadians();

        trenchPID.enableContinuousInput(-Math.PI, Math.PI);
        trenchPID.setTolerance(Math.toRadians(2));

        return (trenchPID.calculate(angle, desiredAngle));
    }
}