package frc.robot.subsystems.Drivetrain;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.system.plant.DCMotor;

import static frc.robot.subsystems.Drivetrain.SwerveDriveConstants.ModuleConstants.kTurningMotorGearRatio;
import static frc.robot.subsystems.Shooter.ShooterConstants.kGearRatio;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;

public class SwerveDriveConstants {
    
    // Enum for closed loop output selection
    public enum ClosedLoopOutputType {
        Voltage,
        TorqueCurrentFOC
    }
    
    public static final class ModuleConstants {
        public static final double kDriveMotorGearRatio = 6.03; // taken from vendor TODO: confirm
        public static final double kTurningMotorGearRatio = (287 / 11.0); // input to output
        public static final double kTurningDegreesToRad = Math.PI / 180.0;
        public static final double kDriveEncoderRot2Rad = (1.0 / kDriveMotorGearRatio) * Math.PI * 2;
        public static final double kTurningEncoderRot2Rad = (1.0 / kTurningMotorGearRatio) * 2 * Math.PI;
        public static final double kDriveEncoderRPM2RadPerSec = kDriveEncoderRot2Rad / 60;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
        
        public static final FeedbackSensorSourceValue kTurnFeedbackSource = FeedbackSensorSourceValue.FusedCANcoder; // Set to RemoteCanCoder if not Fused
        
        // Closed loop output configuration
        public static final ClosedLoopOutputType kDriveClosedLoopOutput = ClosedLoopOutputType.Voltage;
        public static final ClosedLoopOutputType kSteerClosedLoopOutput = ClosedLoopOutputType.Voltage;
    }

    public static final class DriveConstants {

        public static final double kOdometryFrequency = 100.0;

        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 10.0;
        public static final double kWheelRadiusMeters = 0.051; // 1.97 in
        public static final double kSpeedAt12Volts = 5800 * ModuleConstants.kDriveEncoderRPM2RadPerSec * kWheelRadiusMeters; // 5,800RPM freespeed 
        public static final double kPhysicalMaxSpeedMetersPerSecond = kSpeedAt12Volts; 
        public static final CANBus kCANBus = new CANBus("canivore"); // NEEDS TO BE CANIVORE NAME OR SERIAL NUMBER

        public static final int kFrontLeftTurningCanID = 1;
        public static final int kBackLeftTurningCanID = 3;
        public static final int kBackRightTurningCanID = 5;
        public static final int kFrontRightTurningCanID = 7;
        
        public static final int kFrontLeftDriveCanID = 2;
        public static final int kBackLeftDriveCanID = 4;
        public static final int kBackRightDriveCanID = 6;
        public static final int kFrontRightDriveCanID = 8;

        public static final int kFrontLeftAbsoluteEncoderPort = 9;
        public static final int kBackLeftAbsoluteEncoderPort = 10;
        public static final int kBackRightAbsoluteEncoderPort = 11;
        public static final int kFrontRightAbsoluteEncoderPort = 12;

        public static final int kPigeonID = 13;

        public static final boolean kFrontLeftTurningEncoderReversed = false;
        public static final boolean kBackLeftTurningEncoderReversed = false;
        public static final boolean kFrontRightTurningEncoderReversed = false;
        public static final boolean kBackRightTurningEncoderReversed = false;

        public static final boolean kFrontLeftDriveEncoderReversed = false;
        public static final boolean kBackLeftDriveEncoderReversed = true;
        public static final boolean kFrontRightDriveEncoderReversed = false;
        public static final boolean kBackRightDriveEncoderReversed = true;

        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 1.0 ;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond
                / 2.0;
        
        // Linear acceleration limit (m/s²)
        public static final double kTeleDriveMaxAccelerationMetersPerSecSq = 4.5;
        
        // Angular acceleration limit (rad/s²)
        public static final double kTeleDriveMaxAngularAccelerationRadPerSecSq = 12.0;

        public static final double kFrontLeftAbsoluteEncoderOffset = -0.1289; 
        public static final double kBackLeftAbsoluteEncoderOffset = 0.163; 
        public static final double kFrontRightAbsoluteEncoderOffset = -.318;
        public static final double kBackRightAbsoluteEncoderOffset = -0.3;

        // Characterized from drive base radius: driveBaseRadius × 2 / √2 = 21.81 inches
        // Physical measurement: 21.65 inches
        // Using characterized value for better rotation odometry accuracy
        public static final double kTrackWidth = Units.inchesToMeters(21.8);
        public static final double kWheelBase = Units.inchesToMeters(21.8);
        
        // Drive base radius - distance from center to module
        // This is calculated from trackWidth/wheelBase for reference
        // Characterized value: 15.423 inches (from spin test)
        public static final double kDriveBaseRadius = Math.hypot(kTrackWidth / 2.0, kWheelBase / 2.0);


        public static final Translation2d[] moduleTranslations = new Translation2d[] {
                new Translation2d(DriveConstants.kWheelBase / 2.0, DriveConstants.kTrackWidth / 2.0),
                new Translation2d(DriveConstants.kWheelBase / 2.0, -DriveConstants.kTrackWidth / 2.0),
                new Translation2d(-DriveConstants.kWheelBase / 2.0, DriveConstants.kTrackWidth / 2.0),
                new Translation2d(-DriveConstants.kWheelBase / 2.0, -DriveConstants.kTrackWidth / 2.0)
        };

        public static final SwerveDriveKinematics kSwerveKinematics = new SwerveDriveKinematics(moduleTranslations);

        // Drive motor configuration
        public static final int kDriveMotorCurrentLimit = 20;
        public static final DCMotor kDriveGearbox = DCMotor.getKrakenX60(1);

        // Drive PID configuration
        public static final double kDriveKp = 0.0;
        public static final double kDriveKd = 0.0;
        public static final double kDriveKs = 0.08877; // Static friction voltage
        public static final double kDriveKv = 0.10606 ; // V per (rad/s) at motor shaft
        
        public static final double kDriveSimP = 0.05; 
        public static final double kDriveSimD = 0.0;
        public static final double kDriveSimKs = 0.10;  // V - theoretical estimate
        public static final double kDriveSimKv = 0.115;  // V per (rad/s) at WHEEL - theoretical from Kraken X60 specs 

        // Turn motor configuration
        public static final int kTurnMotorCurrentLimit = 40;
        public static final DCMotor kTurnGearbox = DCMotor.getKrakenX44(1);

        // Turn PID configuration
        public static final double kTurnKp = 100.0;
        public static final double kTurnKd = 0.0;
        public static final double kTurnSimP = 9.0;
        public static final double kTurnSimD = 0.0;
        public static final double kTurnPIDMinInput = 0; // Radians
        public static final double kTurnPIDMaxInput = 2 * Math.PI; // Radians

        // PathPlanner configuration
        public static final double kRobotMassKg = 15.88;
        public static final double kRobotMOI = 6.883;
        public static final double kWheelCOF = 1.0;
        public static final RobotConfig robotConfig = new RobotConfig(
                kRobotMassKg,
                kRobotMOI,
                new ModuleConfig(
                        kWheelRadiusMeters,
                        kPhysicalMaxSpeedMetersPerSecond,
                        kWheelCOF,
                        kDriveGearbox.withReduction(ModuleConstants.kDriveMotorGearRatio),
                        kDriveMotorCurrentLimit,
                        1),
                moduleTranslations);
        public static final double PathPlannerDriveP = 3.0;
        public static final double PathPlannerTurnP = 3.0;
        
    }
}
