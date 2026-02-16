package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

/** Shooter subsystem constants */
public final class ShooterConstants {
    
    // Hardware
    public static final int kFlywheelLeaderMotorCanID = 20; // TODO: Set actual CAN ID
    public static final int kFlywheelFollowerMotorCanID = 21; // TODO: Set actual CAN ID
    public static final String kFlywheelCanBus = ""; // Empty for RIO CAN, or "canivore" etc.
    
    // Mechanical
    public static final double kGearRatio = 1.0; // Motor rotations per flywheel rotation
    public static final double kFlywheelMOI = 0.004; // kg*m^2 (moment of inertia)
    public static final double kFlywheelDiameterMeters = 0.1016; // 4 inches - TODO: Update
    public static final double kFlywheelRadiusMeters = kFlywheelDiameterMeters / 2.0;
    
    // Motor configuration
    public static final NeutralModeValue kNeutralMode = NeutralModeValue.Coast;
    public static final InvertedValue kLeaderMotorInverted = InvertedValue.CounterClockwise_Positive;
    
    // Current limits (per motor)
    public static final double kSupplyCurrentLimit = 40.0;
    public static final double kStatorCurrentLimit = 80.0;
    
    // PID gains (Slot 0)
    public static final double kP = 0.1; // TODO: Tune
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    
    // Kraken x44: ~5800 RPM free speed = ~97 RPS @ 12V → kV ≈ 0.12
    public static final double kS = 0.25; // Static friction - TODO: Characterize with SysId
    public static final double kV = 0.12; // Velocity feedforward
    public static final double kA = 0.0; // Acceleration feedforward - TODO: Tune if needed
    
    // Control
    public static final boolean kEnableFOC = true; // Kraken x44 includes FOC
    
    // Tolerances
    public static final double kVelocityToleranceRPM = 150.0;
    public static final double kAtTargetDebounceTime = 0.1; // seconds
    
    // Simulation
    public static final double kSimP = 0.0685;
}
