package frc.robot.ExampleSubsystems.ExampleClimber;

public interface ExampleClimberIO 
{
    public class ExampleClimberIOInputsAutoLogged
    {
        public double examplePositionRotations = 0.0;
        //Motor Position (where the motor currently is)
        public double exampleVelocityRotationsPerSec = 0.0;
        //How fast is it moving currently?
        public double exampleAppliedVoltage = 0.0;
        //Voltage currently being applied to the motor
        public double exampleCurrentAmps = 0.0;
        //current currently being drawn by the motor

        public double setpoint = 0.0;
        //target position the climber is commanded to reach
    }

    default void updateInputs(ExampleClimberIOInputsAutoLogged inputs)
    {
        // Updates the input values with the climber's latest hardware data.
    }

    public default double getExamplePosition()
    {  
        return 0.0; //example getter for retrieving information from the subsystem
    }

    public default void setPID( double kP, double kI, double kD)
    {
        //example setter
        //updates PID gains used to control the subsystem
    }
}
