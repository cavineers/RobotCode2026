package frc.robot.subsystems.Climber;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ClimberConstants {
    public static final boolean kTuningMode = true;
    //enables tuning functionality for the climber mechanism

    public static final int kClimberCanID = 35;
    //Motor ID for the climber motor therefore the code knows which motor it is speaking to

    public static final boolean kEnableFOC = false;
    //Determine wether Fiel Orientation Control is enabled;
    //Field Orientation Control (FOC): A motor-control technique that allows the controller to control brushless motor torque more smoothly and efficiently.

    //Motor Configuration
    //Motor's behavior when no power is being applied
    //Brake mode actively resists movement; Coast mode allows the motor to move freely
    public static final NeutralModeValue kClimberNeutralMode = NeutralModeValue.Brake;
    public static final InvertedValue kClimberMotorInverted = InvertedValue.CounterClockwise_Positive;
    //Determines the positive direction of motor rotation

    public static final double kSupplyCurrentLimit = 20.0; // Amps
    //Maximum continuous current that the motor can draw from the power supply
    public static final double kStatorCurrentLimit = 40.0; // Amps
    //Maximum current allowed through the motor windings

    public static final double kManualSetpointIncrease = 0.5; 
    //Ammount the setpoint changes when manually increased
    public static final double kManualSetpointDecrease = -0.5; 
    //Ammount the setpoint changes when manually decreased

    public static final double kRestMotorRotations = 0.0;
    //Motor Positions measured in rotations
    //Rest Position: climber is fully retracted
    public static final double kDeployedMotorRotations = -60.65;
    //Motor Position once climber is deployed; measured in rotations
    public static final double kEngagedMotorRotations = -23.0;
    //Motor Position once climber is engaged; measured in rotations

    //PID constants used ot control motor position
    public static final double kP = 1.0;
    //P stands for PROPORTIONAL GAIN
    //Determines how strongly the motor responds to position error
    public static final double kI = 0.0; 
    //I stands for INTEGRAL GAIN
    //Corrects for persistent error over time
    public static final double kD = 0.0; 
    //D stands for DERIVATIVE GAIN
    //Responds to how quickly the error is changing and helps reduce overshoot

    //PID Simulation constants
    public static final double kProportionalTermSim = 0.1;
    public static final double kIntegralTermSim = 0.0;
    public static final double kDerivativeTermSim = 0.0;

    public static final boolean kInverted = false;
    //Determines whether the climber's control direction is inverted.
    public static final int kCurrentLimit = 40;
    //General current limit used by the subsystem
    public static final double kTolerance = 0.001;
    //Maximum acceptable difference between the current position and target position.

}
