package frc.robot.subsystems.OverBumperIntake;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class OverBumperIntakeConstants {
    public static final int kDeployMotorCanID = 1; //TODO: set actual ID
    public static final int kIntakeMotorCanID = 2;

    // Motor Configuration
    public static final boolean kInverted = false;
    public static final IdleMode kIdleMode = IdleMode.kBrake;
    public static final int kCurrentLimit = 80;
}