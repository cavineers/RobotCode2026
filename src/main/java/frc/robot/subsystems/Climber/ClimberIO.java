package frc.robot.subsystems.Climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
    @AutoLog
    public static class ClimberIOInputs{
        public double climberPositionRotations = 0.0;
        public double climberVelocityRotationsPerSec = 0.0;
        public double climberAppliedVoltage = 0.0;
        public double climberCurrentAmps = 0.0;

        public double setpoint = 0.0;

        public double[] recentAmpsHistory = new double[20];
        public boolean cutoff = false;
    }
    
    default void updateInputs(ClimberIOInputs inputs) {

    }

    public default void resetEncoder(double rotations) {
        
    }

    public default void updateClimberSetpoint(double rotations){

    }

    public default void setClimberVoltage(double volts) {
    
    }
    
    public default void setPID(double kS, double kV, double kA) {

    } 
}


