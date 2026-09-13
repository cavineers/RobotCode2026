package frc.robot.subsystems.Climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO 
//Interface that will be implemented in the "ClimberIOSim" and "ClimberIOKraken" files
//An interface is a contract that defines what methods a class must provide, allowing different classes to implement those methods in different ways.
{
    @AutoLog
    public static class ClimberIOInputsAutoLogged
    // Stores the inputs and state information reported by the climber.
    {
        public double climberPositionRotations = 0.0;
        //Motor Position (where the motor currently is)
        public double climberVelocityRotationsPerSec = 0.0;
        //How fast is it moving currently?
        public double climberAppliedVoltage = 0.0;
        //Voltage currently being applied to the motor
        public double climberCurrentAmps = 0.0;
        //current currently being drawn by the motor

        public double setpoint = 0.0;
        //target position the climber is commanded to reach
    }
    
    default void updateInputs(ClimberIOInputsAutoLogged inputs) 
    {
        // Updates the input values with the climber's latest hardware data.
    }

    public default void resetEncoder(double rotations) 
    {    
        // Sets the climber's encoder position to the specified number of rotations.
    }

    public default void updateClimberSetpoint(double rotations)
    {
        //Updates the climber's target position to the specified number of rotations.
    }

    public default void setClimberVoltage(double volts) 
    {
        //sets the voltage applied to the motor
    }
    
    public default void setPID(double kS, double kV, double kA) 
    {
        //set PID values
        
        //However, kS, kV, and kA are FeedForward values
        //kS = static friction feedforward
        //kV = velocity feedforward
        //kA = acceleration feedforward

        //When this interface is implemented in the other files, the parameters become the actual PID values
    } 
}


