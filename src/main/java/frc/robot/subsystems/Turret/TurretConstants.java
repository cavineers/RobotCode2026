package frc.robot.subsystems.Turret;

import edu.wpi.first.math.util.Units;

/**
 * Central location for turret configuration values. All numbers here are
 * placeholders and should be updated with measured robot values during the
 * integration phase.
 */
public final class TurretConstants {

    // CAN IDs / hardware
    public static final int kTurretMotorId = 50; // TODO: Update with real CAN ID
    public static final boolean kMotorInverted = false;

    // Gearbox and conversion factors
    public static final double kGearRatio = 20; // TODO: Verify with actual gear ratio

    /** Turret radians per motor rotation. */
    public static final double kPositionConversionFactor = (2.0 * Math.PI) / kGearRatio; // rad / motor rotation

    /** Turret radians per motor RPM. */
    public static final double kVelocityConversionFactor = kPositionConversionFactor / 60.0; // rad / s per motor RPM

    // Mechanical limits
    public static final double kMinAngleRad = 0.0;
    public static final double kMaxAngleRad = Units.degreesToRadians(180.0);

    // Preset positions (robot-relative angles)
    public static final double kPresetOneRad = Units.degreesToRadians(0.0);
    public static final double kPresetTwoRad = Units.degreesToRadians(90.0);
    public static final double kPresetThreeRad = Units.degreesToRadians(150.0);

    // MAXMotion constraints (radians / second and radians / second^2)
    public static final double kMaxMotionCruiseVelocityRadPerSec = Units.degreesToRadians(90.0);
    public static final double kMaxMotionAccelerationRadPerSecSq = Units.degreesToRadians(180.0);
    public static final double kMaxMotionAllowedErrorRad = Units.degreesToRadians(1.0);

    // Electrical limits 
    public static final double kMaxVoltage = 12.0;
    public static final int kCurrentLimitAmps = 30;
    public static final boolean kBrakeModeEnabled = true;

    // PID tuning
    public static final double kPositionKp = 6.0;
    public static final double kPositionKi = 0.0;
    public static final double kPositionKd = 0.4;

    public static final double kPositionToleranceRad = Math.toRadians(1.0);
    public static final double kVelocityToleranceRadPerSec = Math.toRadians(5.0);

    // Homing / limit switch
    public static final boolean kUseHomingSwitch = true;
    public static final boolean kHomingSwitchNormallyOpen = true;
    public static final int kHomingSwitchDioPort = 0; // TODO: Update with actual DIO port
    public static final double kHomingSwitchZeroPositionRad = 0.0;

    // Simulation parameters
    public static final double kSimStartingAngleRad = 0.0;
    public static final double kSimDtSeconds = 0.02; // 20 ms loop time
    public static final double kSimMomentOfInertia = 0.05; // kg * m^2 (placeholder)
    public static final double kSimFrictionTorquePerRadPerSec = 0.02; // N*m per rad/s of velocity
    public static final double kSimBrakeTorquePerRadPerSec = 0.08; // additional damping when brake mode enabled
}
