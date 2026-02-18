package frc.robot.subsystems.InBumperIntake;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class InBumperIntakeConstants {

    public static final int kBottomMotorCanID = 1; //TODO: set actual ids
    public static final int kTopMotorCanID = 2;
    public static final int kOutsideMotorCanID = 3;

    // Motor Configuration
    public static final boolean kTopInverted = true;
    public static final IdleMode kTopIdleMode = IdleMode.kBrake;
    public static final int kTopCurrentLimit = 40;

    public static final boolean kBottomInverted = false;
    public static final IdleMode kBottomIdleMode = IdleMode.kBrake;
    public static final int kBottomCurrentLimit = 40;

    public static final boolean kOutsideInverted = true;
    public static final IdleMode kOutsideIdleMode = IdleMode.kBrake;
    public static final int kOutsideCurrentLimit = 40;

    public static final double kDefaultVoltage = 12.0;
    public static final double kBottomVoltage = kDefaultVoltage * 0.65; //TODO: set speed
    public static final double kOutsideVoltage = kDefaultVoltage * 0.65;
    public static final double kTopVoltage = kDefaultVoltage * 0.65;
}
