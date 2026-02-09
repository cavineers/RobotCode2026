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

}
