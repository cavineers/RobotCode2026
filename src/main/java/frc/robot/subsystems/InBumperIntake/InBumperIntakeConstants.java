package frc.robot.subsystems.InBumperIntake;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class InBumperIntakeConstants {

    public static final int kBottomMotorCanID = 21; //TODO: set actual ids
    public static final int kTopMotorCanID = 22;
    public static final int kOutsideMotorCanID = 20;
    public static final int kSpindexerMotorCanID = 23; //TODO: set actual id

    // Motor Configuration
    public static final boolean kTopInverted = false;
    public static final IdleMode kTopIdleMode = IdleMode.kBrake;
    public static final int kTopCurrentLimit = 40;

    public static final boolean kBottomInverted = true;
    public static final IdleMode kBottomIdleMode = IdleMode.kBrake;
    public static final int kBottomCurrentLimit = 40;

    public static final boolean kOutsideInverted = false;
    public static final IdleMode kOutsideIdleMode = IdleMode.kBrake;
    public static final int kOutsideCurrentLimit = 40;

    public static final boolean kSpindexerInverted = false;
    public static final IdleMode kSpindexerIdleMode = IdleMode.kCoast;
    public static final int kSpindexerCurrentLimit = 30;

    public static final double kDefaultVoltage = 12.0;
    public static final double kBottomVoltage = kDefaultVoltage * 0.6; //TODO: set speed
    public static final double kOutsideVoltage = kDefaultVoltage * 0.6;
    public static final double kTopVoltage = kDefaultVoltage * 0.6;
    public static final double kSpindexerVoltage = kDefaultVoltage * 0.5; //TODO: tune speed
}
