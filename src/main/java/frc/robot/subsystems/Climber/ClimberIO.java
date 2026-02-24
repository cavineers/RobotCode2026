package frc.robot.subsystems.Climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
    @AutoLog
    public static class ClimberIOInputs{
        public double climberPositionRotations = 0.0;
        public double climberVelocityRotationsPerSec = 0.0;
        public double climberAppliedVoltage = 0.0;
        public double climberCurrentAmps = 0.0;

        public double[] recentAmpsHistory = new double[20];
        public boolean cutoff = false;
    }
    
    default void updateInputs(ClimberIOInputs inputs) {

    }

    public default void resetEncoder(double rotations) {
        
    }

    public default void setClosedLoop(boolean isClosed){

    }

    public default void updateClimberSetpoint(double rotations){

    }

    public default void deploy() {

    }

    public default void retract() {

    }

    public default void engage(){
        
    }

    public default void setClimberVoltage(double volts) {
    
    }
    
    public default void setPID(double kS, double kV, double kA) {

    }
    
    public default void setFF(double kS, double kV, double kA) {

    }
}


