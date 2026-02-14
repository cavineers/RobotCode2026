package frc.robot.subsystems.Drivetrain;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.system.plant.DCMotor;

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

        public static final double kPhysicalMaxSpeedMetersPerSecond = 4.07; 
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = Units.degreesToRadians(720);
        public static final double kWheelRadiusMeters = Units.inchesToMeters(2.0);
        public static final double kSpeedAt12Volts = 5800 * ModuleConstants.kDriveEncoderRPM2RadPerSec * kWheelRadiusMeters; // 5,800RPM freespeed 
        public static final CANBus kCANBus = new CANBus("canivore"); // NEEDS TO BE CANIVORE NAME OR SERIAL NUMBER

        public static final int kFrontLeftTurningCanID = 8;
        public static final int kFrontRightTurningCanID = 2;
        public static final int kBackRightTurningCanID = 4;
        public static final int kBackLeftTurningCanID = 6;
        
        public static final int kFrontLeftDriveCanID = 7;
        public static final int kFrontRightDriveCanID = 1;
        public static final int kBackRightDriveCanID = 3;
        public static final int kBackLeftDriveCanID = 5;

        public static final int kFrontLeftAbsoluteEncoderPort = 12;
        public static final int kFrontRightAbsoluteEncoderPort = 11;
        public static final int kBackRightAbsoluteEncoderPort = 10;
        public static final int kBackLeftAbsoluteEncoderPort = 9;

        public static final int kPigeonID = 13;

        public static final boolean kFrontLeftTurningEncoderReversed = false;
        public static final boolean kBackLeftTurningEncoderReversed = false;
        public static final boolean kFrontRightTurningEncoderReversed = false;
        public static final boolean kBackRightTurningEncoderReversed = false;

        public static final boolean kFrontLeftDriveEncoderReversed = false;
        public static final boolean kBackLeftDriveEncoderReversed = false;
        public static final boolean kFrontRightDriveEncoderReversed = false;
        public static final boolean kBackRightDriveEncoderReversed = false;

        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond
                / 2.0;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3.5;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 2.5;

        public static final double kFrontLeftAbsoluteEncoderOffset = 0; // In rotations (read straight from the encoder)
        public static final double kBackLeftAbsoluteEncoderOffset = 0; // This is done by zeroing in cancoder
        public static final double kFrontRightAbsoluteEncoderOffset = 0;
        public static final double kBackRightAbsoluteEncoderOffset = 0;

        // Distance between right and left wheefls
        public static final double kTrackWidth = Units.inchesToMeters(24);
        // Distance between front and back wheels
        public static final double kWheelBase = Units.inchesToMeters(25);
        public static final double kDriveBaseRadius = Math.hypot(kTrackWidth / 2.0, kWheelBase / 2.0);
        public static final double kSideLength = Units.inchesToMeters(17.5); // 2.5 inches bumper: Represents full width
                                                                             // of the robot

        public static final Translation2d[] moduleTranslations = new Translation2d[] {
                new Translation2d(DriveConstants.kWheelBase / 2.0, DriveConstants.kTrackWidth / 2.0),
                new Translation2d(DriveConstants.kWheelBase / 2.0, -DriveConstants.kTrackWidth / 2.0),
                new Translation2d(-DriveConstants.kWheelBase / 2.0, DriveConstants.kTrackWidth / 2.0),
                new Translation2d(-DriveConstants.kWheelBase / 2.0, -DriveConstants.kTrackWidth / 2.0)
        };

        public static final SwerveDriveKinematics kSwerveKinematics = new SwerveDriveKinematics(moduleTranslations);

        // Drive motor configuration
        public static final int kDriveMotorCurrentLimit = 30;
        public static final DCMotor kDriveGearbox = DCMotor.getNEO(1);

        // Drive PID configuration
        public static final double kDriveKp = 0.0;
        public static final double kDriveKd = 0.0;
        public static final double kDriveKs = 0.;
        public static final double kDriveKv = 0.14043;
        
        public static final double kDriveSimP = 1;
        public static final double kDriveSimD = 0.0;
        public static final double kDriveSimKs = 0.01370;
        public static final double kDriveSimKv = 0.13394;

        // Turn motor configuration
        public static final int kTurnMotorCurrentLimit = 40;
        public static final DCMotor kTurnGearbox = DCMotor.getNEO(1);

        // Turn PID configuration
        public static final double kTurnKp = 0.5;
        public static final double kTurnKd = 0.0;
        public static final double kTurnSimP = 9.0;
        public static final double kTurnSimD = 0.0;
        public static final double kTurnPIDMinInput = 0; // Radians
        public static final double kTurnPIDMaxInput = 2 * Math.PI; // Radians

        // PathPlanner configuration
        public static final double kRobotMassKg = 56.7;
        public static final double kRobotMOI = 6.883;
        public static final double kWheelCOF = 1.0;
        public static final RobotConfig robotConfig = new RobotConfig(
                kRobotMassKg,
                kRobotMOI,
                new ModuleConfig(
                        kWheelRadiusMeters,
                        kPhysicalMaxSpeedMetersPerSecond,
                        kWheelCOF,
                        kDriveGearbox.withReduction(1 / ModuleConstants.kDriveMotorGearRatio),
                        kDriveMotorCurrentLimit,
                        1),
                moduleTranslations);
        public static final double PathPlannerDriveP = 3.0;
        public static final double PathPlannerTurnP = 5.0;
        
    }
}
