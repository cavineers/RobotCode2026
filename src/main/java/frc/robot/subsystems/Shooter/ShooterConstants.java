package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

/**
 * @brief Constants for the shooter subsystem.
 */
public final class ShooterConstants {
    
    // Hardware
    public static final int kFlywheelMotorCanID = 20; // TODO: Set actual CAN ID
    public static final String kFlywheelCanBus = ""; // Empty string for rio CAN, or "canivore" etc.
    
    // Mechanical
    public static final double kGearRatio = 1.0; // motor rotations per flywheel rotation
    public static final double kFlywheelMOI = 0.004; // kg*m^2 (moment of inertia)
    public static final double kFlywheelDiameterMeters = 0.1016; // 4 inches in meters - TODO: Update to actual diameter
    public static final double kFlywheelRadiusMeters = kFlywheelDiameterMeters / 2.0;
    
    // Motor configuration
    public static final NeutralModeValue kNeutralMode = NeutralModeValue.Coast;
    public static final InvertedValue kMotorInverted = InvertedValue.CounterClockwise_Positive;
    
    // Current limits
    public static final double kSupplyCurrentLimit = 40.0; // Amps
    public static final double kStatorCurrentLimit = 80.0; // Amps
    
    // PID gains (Slot 0)
    public static final double kP = 0.1; // TODO: Tune
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    
    // Feedforward gains
    public static final double kS = 0.0; // Static friction (V)
    public static final double kV = 0.12; // Velocity feedforward (V/(rot/s)) - 12V / 100 RPS = 0.12
    public static final double kA = 0.0; // Acceleration feedforward (V/(rot/s^2))
    
    // Control
    public static final boolean kEnableFOC = false; // Set to true if you have the FOC upgrade ($150)
    
    // Tolerances
    public static final double kVelocityToleranceRPM = 150.0; // RPM tolerance for "at target"
    public static final double kAtTargetDebounceTime = 0.1; // seconds
    
    // Simulation
    public static final double kSimP = 0.0685; // Simple P gain for sim
}
