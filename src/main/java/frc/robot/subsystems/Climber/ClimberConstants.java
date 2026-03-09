package frc.robot.subsystems.Climber;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ClimberConstants {
    public static final boolean kTuningMode = true;

    public static final int kClimberCanID = 35; //TODO: Update

    public static final boolean kEnableFOC = false;
    // Motor configuration Kraken X60
    public static final NeutralModeValue kClimberNeutralMode = NeutralModeValue.Brake;
    public static final InvertedValue kClimberMotorInverted = InvertedValue.CounterClockwise_Positive;

    public static final double kSupplyCurrentLimit = 20.0; // Amps
    public static final double kStatorCurrentLimit = 40.0; // Amps

    public static final double kManualSetpointIncrease = 0.5; //TODO: Update
    public static final double kManualSetpointDecrease = -0.5; //TODO: Update

    public static final double kRestMotorRotations = 0.0;
    public static final double kDeployedMotorRotations = 42.0;
    public static final double kEngagedMotorRotations = 10.0;

    public static final double kP = 1.0; //TODO: Update
    public static final double kI = 0.0; //TODO: Update
    public static final double kD = 0.0; //TODO: Update

    public static final double kProportionalTermSim = 0.1;
    public static final double kIntegralTermSim = 0.0;
    public static final double kDerivativeTermSim = 0.0;

    public static final boolean kInverted = false;
    public static final int kCurrentLimit = 40;
    public static final double kTolerance = 0.001;

}
