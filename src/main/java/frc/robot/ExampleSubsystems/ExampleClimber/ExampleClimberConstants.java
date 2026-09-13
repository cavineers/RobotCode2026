package frc.robot.ExampleSubsystems.ExampleClimber;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ExampleClimberConstants {
    //GENERAL CONFIGURATION

    //Motor CAN ID
    public static final int kMotorCanID = 1;

    //Is Field Orientation Control enabled?
    public static final boolean kEnableFOC = false;

    //Motor behavior when no power is applied
    public static final NeutralModeValue kNeutralMode = NeutralModeValue.Brake;

    //Motor rotation direction
    public static final InvertedValue kMotorInverted = InvertedValue.CounterClockwise_Positive;



    //CURRENT LIMITS

    //Maximum continuous current supplied to the motor
    public static final double kSupplyCurrentLimit = 0.0;
    //Maximum current through the motor windings
    public static final double kStatorCurrentLimit = 0.0;

    //POSITION SETPOINTS

    //Example target positions, measured in rotations
    public static final double kDeployedMotorRotations = 0.0;
    public static final double kRestMotorRotations = 0.0;

    //PID CONSTANTS

    //Proportional, Integral, and Derivative Gains
    public static final double kP = 0.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    //CONTROL SETTINGS
    
    //Maximum acceptable position error
    public static final double kTolerance = 0.0;
}
