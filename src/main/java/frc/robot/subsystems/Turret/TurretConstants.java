package frc.robot.subsystems.Turret;

import edu.wpi.first.math.util.Units;

/**
 * Central location for turret configuration values. All numbers here are
 * placeholders and should be updated with measured robot values during the
 * integration phase.
 */
public final class TurretConstants {

    /** Selects which homing strategy to use. */
    public enum HomingMethod {
        /** Detect the mechanical hardstop via a sustained motor current spike. */
        CURRENT_SPIKE,
        /** Detect the home position via a physical limit switch on a DIO port. */
        LIMIT_SWITCH
    }

    /** Active homing method — change this to switch between strategies. */
    public static final HomingMethod kHomingMethod = HomingMethod.CURRENT_SPIKE;

    // CAN IDs / hardware
    public static final int kTurretMotorId = 51; // TODO: Update with real CAN ID
    public static final boolean kMotorInverted = true;

    // Gearbox and conversion factors
    public static final double kGearRatio = 10; // TODO: Verify with actual gear ratio

    /** Turret radians per motor rotation. */
    public static final double kPositionConversionFactor = (2.0 * Math.PI) / kGearRatio; // rad / motor rotation

    /** Turret radians per motor RPM. */
    public static final double kVelocityConversionFactor = kPositionConversionFactor / 60.0; // rad / s per motor RPM

    // Mechanical limits
    // Defined as Robot-Relative angles (0 is Front/North)
    public static final double kMinAngleRad = Units.degreesToRadians(-210.0);
    public static final double kMaxAngleRad = Units.degreesToRadians(150.0);
    // Offset from the robot's front (0 degrees) to the turret's "Mechanical Zero"
    public static final double kTurretZeroOffsetRad = 0.0; // Encoder zero is at robot front (north)
    // Preset positions (robot-relative angles)
    public static final double kPresetOneRad = Units.degreesToRadians(0.0); // Front
    public static final double kPresetTwoRad = Units.degreesToRadians(45.0); // Left Diagonal
    public static final double kPresetThreeRad = Units.degreesToRadians(-45.0); // Right Diagonal

    // Electrical limits 
    public static final double kMaxVoltage = 12.0;
    public static final int kCurrentLimitAmps = 30;
    public static final boolean kBrakeModeEnabled = true;

    // PID tuning
    public static final double kPositionKp = 0.45; // Reduced from 2.0 to prevent oscillation on low-inertia test rig
    public static final double kPositionKi = 0.0;
    public static final double kPositionKd = 0.0;

    public static final double kPositionToleranceRad = Math.toRadians(1.0);
    public static final double kVelocityToleranceRadPerSec = Math.toRadians(5.0);

    // Moving camera support - turret angle history buffer
    public static final double kTurretAngleBufferSizeSec = 2.0; // Store 2 seconds of turret angle history

    // Homing / limit switch
    public static final boolean kHomingSwitchNormallyOpen = false;
    public static final int kHomingSwitchDioPort = 4; // TODO: Update with actual DIO port
    public static final double kHomingSwitchZeroPositionRad = 0;
    public static final double kHomingSearchMaxVoltage = 3.0; // Reduced voltage when not homed for safety

    // Current-based homing (hardstop detection)
    public static final double kHomingVoltage = -1.25; // Voltage to apply during homing
    public static final double kHomingCurrentThresholdAmps = 25; // Current threshold to detect hardstop
    public static final int kHomingCurrentSpikeCountRequired = 10; // Number of consecutive readings above threshold (at 50Hz = 0.1s)
    public static final double kHomingHardstopPositionRad = kMaxAngleRad; // -5.236 rad - encoder resets to this at hardstop

    // Simulation parameters
    public static final double kSimStartingAngleRad = 0.0; // Start at 0 rad (front of robot)
    public static final double kSimDtSeconds = 0.02; // 20 ms loop time
    public static final double kSimMomentOfInertia = 0.05; // kg * m^2 (placeholder)
    public static final double kSimFrictionTorquePerRadPerSec = 0.02; // N*m per rad/s of velocity
    public static final double kSimBrakeTorquePerRadPerSec = 0.08; // additional damping when brake mode enabled
}
