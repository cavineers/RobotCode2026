package frc.robot.subsystems.Climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
    @AutoLog
    public static class ClimberIOInputs{
        public double deployMotorPositionRotations = 0.0;
        public double deployMotorVelocityRadPerSec = 0.0;
        public double deployMotorAppliedVolts = 0.0;
        public double deployMotorCurrentAmps = 0.0;

        public double intakeMotorPositionRotations = 0.0;
        public double intakeMotorVelocityRadPerSec = 0.0;
        public double intakeMotorAppliedVolts = 0.0;
        public double intakeMotorCurrentAmps = 0.0;
        public boolean deployed = false;
    }
    
    default void updateInputs(ClimberIOInputs inputs) {
    }

    public default void deploy() {
    }

    public default void retract() {
    }

}


