package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

/** Shooter subsystem constants */
public final class ShooterConstants {
    
    // Hardware
    public static final int kFlywheelLeaderMotorCanID = 31; // TODO: Set actual CAN ID
    public static final int kFlywheelFollowerMotorCanID = 32; // TODO: Set actual CAN ID
    public static final String kFlywheelCanBus = "canivore"; // Empty for RIO CAN, or "canivore" etc.
    
    // Mechanical
    // 1:1.33 upduction — flywheel spins 1.33× faster than motor shaft.
    // kGearRatio = motor rotations per flywheel rotation = 1/1.33 (used by Kraken IO for unit conversion)
    public static final double kGearRatio = 1.0;
    // kSimGearRatio = flywheel rotations per motor rotation = 1.33 (used by DCMotorSim — WPILib convention is output/input)
    public static final double kSimGearRatio = 1.0;
    public static final double kFlywheelMOI = 0.004; // kg*m^2 (moment of inertia)
    public static final double kFlywheelDiameterMeters = 0.0762; // 3 inches - TODO: Update
    public static final double kFlywheelRadiusMeters = kFlywheelDiameterMeters / 2.0;
    
    // Motor configuration
    public static final NeutralModeValue kNeutralMode = NeutralModeValue.Coast;
    public static final InvertedValue kLeaderMotorInverted = InvertedValue.Clockwise_Positive;
    
    // Current limits (per motor)
    public static final double kSupplyCurrentLimit = 40.0;
    public static final double kStatorCurrentLimit = 80.0;
    
    // PID gains (Slot 0) - Aggressive response with high P
    // kV feedforward provides steady-state control, kP provides error correction
    public static final double kP = 0.1;  // High P for fast response (clamped to 12V by TalonFX)
    public static final double kI = 0.0;  // No integral to avoid lag and windup
    public static final double kD = 0.0;  // Feedforward dominates, so minimal derivative needed
    
    // PID gains (Slot 1) - Feedforward only (kP=0) for deadband phase
    // Used when within error deadband - only feedforward runs on motor controller's high-frequency loop
    public static final double kP_FFOnly = 0.0;    // Zero P to disable PID in deadband
    public static final double kI_FFOnly = 0.0;    // No integral
    public static final double kD_FFOnly = 0.0;    // No derivative
    
    // When error is within this deadband, Slot 1 (kP=0) is used, letting feedforward dominate
    public static final double kVelocityErrorDeadbandRPM = 100.0;
    
    // Feedforward gains (both slots use these)
    public static final double kS = 0.19832; // Static friction - TODO: Characterize with SysId
    public static final double kV = 0.09380; // Velocity feedforward
    public static final double kA = 0.0;     // Acceleration feedforward - TODO: Tune if needed
    
    // Control
    public static final boolean kEnableFOC = false; // Kraken x44 includes FOC
    
    // Tolerances
    public static final double kVelocityToleranceRPM = 150.0;
    public static final double kAtTargetDebounceTime = 0.1; // seconds

    // Manual mode presets
    public static final double kManualMidrangeRPM = 4200.0;
    public static final double kManualCloseRPM = 3200.0;
    
    // Simulation
    public static final double kSimP = 0.0;
    public static final double kSimKS = 0.10;
    // Kraken X44 free speed ~97 motor RPS @ 12V -> kV = 12 / (97 * 2π)
    public static final double kSimKV = 12.0 / (97.0 * 2.0 * Math.PI);
}
