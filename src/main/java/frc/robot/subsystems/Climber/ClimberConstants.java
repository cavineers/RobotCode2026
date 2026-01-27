package frc.robot.subsystems.Climber;

public class ClimberConstants {
    public static final boolean kTuningMode = true;

    public static final int kClimberCanID = 51;

    public static final double kRestAbsoluteRotations = 0.0;
    public static final double kDeployedAbsoluteRotations = 5.0; //TODO: Update these 3
    public static final double kEngagedAbsoluteRotations = 2.5;
    public static final double kClimberGearRatio = 5.0;

    public static final double kRestMotorRotations = kClimberGearRatio * kRestAbsoluteRotations;
    public static final double kDeployedMotorRotations = kClimberGearRatio * kDeployedAbsoluteRotations;
    public static final double kEngagedMotorRotations = kClimberGearRatio * kEngagedAbsoluteRotations;

    public static final double kProportionalGainSpark = 1.0;
    public static final double kIntegralTermSpark = 0.0;
    public static final double kDerivativeTermSpark = 1.0;
    public static final double kGravityTermSpark = -0.6;

    public static final double kProportionalTermSim = 0.1;
    public static final double kDerivativeTermSim = 0.0;

    public static final boolean kInverted = false;
    public static final int kCurrentLimit = 40;
    public static final double kTolerance = 0.001;

}
