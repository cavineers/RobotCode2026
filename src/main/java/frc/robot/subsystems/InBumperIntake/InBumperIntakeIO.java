package frc.robot.subsystems.InBumperIntake;

import org.littletonrobotics.junction.AutoLog;

public interface InBumperIntakeIO {
    @AutoLog
    public static class InBumperIntakeIOInputs {
        public double bottomMotorPositionRotations = 0.0;
        public double bottomMotorVelocityRPM = 0.0;
        public double bottomMotorAppliedVolts = 0.0;
        public double bottomMotorCurrentAmps = 0.0;

        public double topMotorPositionRotations = 0.0;
        public double topMotorVelocityRPM = 0.0;
        public double topMotorAppliedVolts = 0.0;
        public double topMotorCurrentAmps = 0.0;

        public double outsideMotorPositionRotations = 0.0;
        public double outsideMotorVelocityRPM = 0.0;
        public double outsideMotorAppliedVolts = 0.0;
        public double outsideMotorCurrentAmps = 0.0;

        public double indexerMotorPositionRotations = 0.0;
        public double indexerMotorVelocityRPM = 0.0;
        public double indexerMotorAppliedVolts = 0.0;
        public double indexerMotorCurrentAmps = 0.0;

        public double rollerMotorPositionRotations = 0.0;
        public double rollerMotorVelocityRPM = 0.0;
        public double rollerMotorAppliedVolts = 0.0;
        public double rollerMotorCurrentAmps = 0.0;
        public boolean rollerConnected = false;

    }
    
    public default void updateInputs(InBumperIntakeIOInputs inputs) {}
    
    public default void setBottomVoltage(double volts) {}

    public default void setTopVoltage(double volts) {}

    public default void setOutsideVoltage(double volts) {}

    public default void setIndexerVoltage(double volts) {}

    public default void setRollerVoltage(double volts) {}
} 