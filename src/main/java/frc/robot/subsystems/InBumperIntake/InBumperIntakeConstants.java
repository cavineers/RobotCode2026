package frc.robot.subsystems.InBumperIntake;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class InBumperIntakeConstants {

    public static final int kBottomMotorCanID = 21; //TODO: set actual ids
    public static final int kTopMotorCanID = 22;
    public static final int kOutsideMotorCanID = 20;
    public static final int kIndexerMotorCanID = 23; //TODO: set actual id

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

    public static final boolean kIndexerInverted = true;
    public static final IdleMode kIndexerIdleMode = IdleMode.kBrake;
    public static final int kIndexerCurrentLimit = 30;

    //Kraken
    public static final boolean kEnableFOC = true; // Kraken x44 includes FOC
    public static final NeutralModeValue kNeutralMode = NeutralModeValue.Coast;
    public static final InvertedValue kRollerInverted = InvertedValue.Clockwise_Positive;
    public static final double kSupplyCurrentLimit = 40.0;
    public static final double kStatorCurrentLimit = 80.0;
    public static final int kRollerCanID = 35; 
    public static final String kRollerCanBus = "canivore"; // Empty for RIO CAN, or "canivore" etc.

    public static final double kDefaultVoltage = 12.0;
    public static final double kBottomVoltage = kDefaultVoltage * 0.6; //TODO: set speed
    public static final double kOutsideVoltage = kDefaultVoltage * 0.6;
    public static final double kTopVoltage = kDefaultVoltage * 0.6;
    public static final double kIndexerVoltage = kDefaultVoltage * 0.6; //TODO: tune speed
    public static final double kRollerVoltage = kDefaultVoltage * 0.6;
}
