package frc.robot.subsystems.OverBumperIntake;

import org.littletonrobotics.junction.AutoLog;

public interface OverBumperIntakeIO {
    @AutoLog
    public static class OverBumperIntakeIOInputs {
        public double deployPositionRad = 0.0;
        public double deployVelocityRadPerSec = 0.0;
        public double deployAppliedVolts = 0.0;
        public double deployCurrentAmps = 0.0;
        public boolean deployed = false;
        public double[] recentAmpsHistory = new double[20];

        public double intakePositionRad = 0.0;
        public double intakeVelocityRadPerSec = 0.0;
        public double intakeAppliedVolts = 0.0;
        public double intakeCurrentAmps = 0.0;
    }

    
    public default void updateInputs(OverBumperIntakeIOInputs inputs) {}

    public default void deploy() {}

    public default void retract() {}

    public default void intake() {}

    public default void outtake() {}

    public default void setIntakeVoltage(double volts) {}

    public default void setDeployVoltage(double volts) {}

    public default void setPID(double kp, double ki, double kd) {}

    public default void setFF(double ks, double kv, double ka) {}
}