package frc.robot.subsystems.InBumperIntake;

import org.littletonrobotics.junction.AutoLog;

public interface InBumperIntakeIO {
    @AutoLog
    public static class InBumperIntakeIOInputs {
        public double bottomMotorPositionRad = 0.0;
        public double bottomMotorVelocityRadPerSec = 0.0;
        public double bottomMotorAppliedVolts = 0.0;
        public double bottomMotorCurrentAmps = 0.0;

        public double topMotorPositionRad = 0.0;
        public double topMotorVelocityRadPerSec = 0.0;
        public double topMotorAppliedVolts = 0.0;
        public double topMotorCurrentAmps = 0.0;

        public double outsideMotorPositionRad = 0.0;
        public double outsideMotorVelocityRadPerSec = 0.0;
        public double outsideMotorAppliedVolts = 0.0;
        public double outsideMotorCurrentAmps = 0.0;

        public double spindexerMotorPositionRad = 0.0;
        public double spindexerMotorVelocityRadPerSec = 0.0;
        public double spindexerMotorAppliedVolts = 0.0;
        public double spindexerMotorCurrentAmps = 0.0;

    }
    
    public default void updateInputs(InBumperIntakeIOInputs inputs) {}
    
    public default void setBottomVoltage(double volts) {}

    public default void setTopVoltage(double volts) {}

    public default void setOutsideVoltage(double volts) {}

    public default void setSpindexerVoltage(double volts) {}
} 