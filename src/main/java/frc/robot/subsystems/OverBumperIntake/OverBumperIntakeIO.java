package frc.robot.subsystems.OverBumperIntake;

import org.littletonrobotics.junction.AutoLog;

public interface OverBumperIntakeIO {
    @AutoLog
    public static class OverBumperIntakeIOInputs {
        public double deployPositionRotations = 0.0;
        public double deployVelocityRotations = 0.0;
        public double deployAppliedVolts = 0.0;
        public double deployCurrentAmps = 0.0;
        public boolean deployed = false;
        public double[] recentAmpsHistory = new double[20];
        public double deployVelocityRotationsPerSec = 0.0;

        public double intakePositionRotations = 0.0;
        public double intakeVelocityRotPerSec = 0.0;
        public double intakeAppliedVolts = 0.0;
        public double intakeCurrentAmps = 0.0;
        public double intakeVelocityRotationsPerSec = 0.0;
        public boolean cutoff = false;
        public boolean isClosed = false;
    }
    
    public default void updateInputs(OverBumperIntakeIOInputs inputs) {}

    public default void autoDeploy() {}

    public default void intake() {}

    public default void outtake() {}

    public default void updateSetpoint(double setpoint) {}

    public default void setClosedLoop(boolean val){}

    public default void resetEncoder(double positionRad){}

    public default void setIntakeVoltage(double volts) {}

    public default void setDeployVoltage(double volts) {}

    public default void setPID(double kp, double ki, double kd) {}
}