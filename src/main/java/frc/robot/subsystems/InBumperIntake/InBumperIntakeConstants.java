package frc.robot.subsystems.InBumperIntake;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class InBumperIntakeConstants {

    public static final int kBottomMotorCanID = 1; //TODO: set actual ids
    public static final int kHopperMotorCanID = 2;
    public static final int kTopMotorCanID = 3;

    // Motor Configuration
    public static final boolean kInverted = false;
    public static final IdleMode kIdleMode = IdleMode.kBrake;
    public static final int kCurrentLimit = 80;
    public static final double kBottomVoltage = 0.3;
    public static final double kTopVoltage = 0.3;
    public static final double kHopperVoltage = 0.3;
    public static final double kCutOffAmps = 40.0; //TODO: set value

}
