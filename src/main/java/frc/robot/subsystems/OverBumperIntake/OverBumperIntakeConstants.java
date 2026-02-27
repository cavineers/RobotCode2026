package frc.robot.subsystems.OverBumperIntake;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class OverBumperIntakeConstants {
    public static final int kDeployMotorCanID = 30; //TODO: set actual ID
    public static final int kIntakeMotorCanID = 31;

    // Motor Configuration
    public static final boolean kInverted = true;
    public static final IdleMode kIdleMode = IdleMode.kBrake;
    public static final int kCurrentLimit = 80;
    
    public static final double kIntakeVoltage = 0.5;
    public static final double kDeployVoltage = 0.1;

    public static final double kDeployedRotations = -2.88;
    public static final double kRetractedRotations = 0.0; //TODO: set values

    //PID Gains (Slot 0)
    public static final double kProportionalGainSpark = 1.2; //TODO: tune
    public static final double kIntegralTermSpark = 0.0;
    public static final double kDerivativeTermSpark = 0.025;

    //Simulation Constants
    public static final double kSimP = 0.55; //TODO: tune
    public static final double kSimI = 0.0;
    public static final double kSimD = 0.3;
}