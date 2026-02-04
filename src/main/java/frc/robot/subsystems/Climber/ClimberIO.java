package frc.robot.subsystems.Climber;

import frc.robot.subsystems.Climber.*;
import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
    @AutoLog
    public static class ClimberIOInputs{
        public double climberPositionRotations = 0.0;
        public double climberVelocityRotationsPerSec = 0.0;
        public double climberAppliedVoltage = 0.0;
        public double climberCurrentAmps = 0.0;

        public boolean deployed = false;
    }
    
    default void updateInputs(ClimberIOInputs inputs) {

    }

    public default void setClosedLoop(boolean isClosed){

    }

    public default void updateClimberPosition(double rotations){

    }

    public default void deploy() {

    }

    public default void retract() {

    }

    public default void setDeployVoltage(double volts) {
    
    }

    public default void setRetractVoltage(double volts) {
    
    }
}


