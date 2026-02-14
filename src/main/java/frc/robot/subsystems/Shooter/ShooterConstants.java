package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class ShooterConstants {
    public static final int kFlywheelCanID = 1; //TODO: Set to actual CanID
    public static final int kFollowerCanID = 3;
    public static final int kAngleCanID = 2;
    public static final String kShooterCanBus = "";

    public static final int kShooterIR = 3;

    public static final boolean kEnableFOC = false;

    // Mechanical
    public static final double kGearRatio = 1.0; // motor rotations per flywheel rotation
    public static final double kFlywheelMOI = 0.004; // kg*m^2 (moment of inertia)
    public static final double kFlywheelDiameterMeters = 0.1016; // 4 inches in meters - TODO: Update to actual diameter
    public static final double kFlywheelRadiusMeters = kFlywheelDiameterMeters / 2.0;

    // Motor configuration Kraken X60
    public static final NeutralModeValue kFlywheelNeutralMode = NeutralModeValue.Coast;
    public static final InvertedValue kFlywheelMotorInverted = InvertedValue.CounterClockwise_Positive;
    public static final NeutralModeValue kFollowerNeutralMode = NeutralModeValue.Coast;
    public static final InvertedValue kFollowerMotorInverted = InvertedValue.CounterClockwise_Positive;
    
    public static final double kSupplyCurrentLimit = 20.0; // Amps
    public static final double kStatorCurrentLimit = 40.0; // Amps

    // PID gains (Slot 0)
    public static final double kP = 0.1; // TODO: Tune
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    // Feedforward gains
    public static final double kS = 0.0; // Static friction (V)
    public static final double kV = 0.12; // Velocity feedforward (V/(rot/s)) - 12V / 100 RPS = 0.12
    public static final double kA = 0.0; // Acceleration feedforward (V/(rot/s^2))

    // Simulation
    public static final double kSimP = 0.0685; // Simple P gain for sim

    // Motor Configuration NEO
    public static final boolean kInverted = false;
    public static final IdleMode kIdleMode = IdleMode.kBrake;
    public static final int kCurrentLimit = 40;

}
