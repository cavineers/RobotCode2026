package frc.robot.subsystems.OverBumperIntake;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class OverBumperIntakeConstants {
    public static final int kDeployMotorCanID = 50; //TODO: set actual ID
    public static final int kIntakeMotorCanID = 51;

    // Motor Configuration
    public static final boolean kInverted = false;
    public static final IdleMode kIdleMode = IdleMode.kBrake;
    public static final int kCurrentLimit = 80;
    
    public static final double kIntakeVoltage = 0.1;
    public static final double kDeployVoltage = 0.1;
    public static final double kCutOffAmps = 7.05; //TODO: set value

    public static final double kDeployedRotations = 5.0;
    public static final double kRetractedRotations = 0.0; //TODO: set values
    public static final double kHomingSwitchZeroPositionRot = 5.0;

    //PID Gains (Slot 0)
    public static final double kProportionalGainSpark = 1.7; //TODO: tune
    public static final double kIntegralTermSpark = 0.0;
    public static final double kDerivativeTermSpark = 0.0;

    //Simulation Constants
    public static final double kSimP = 0.047; //TODO: tune
    public static final double kSimI = 0.0;
    public static final double kSimD = 0.1;
}