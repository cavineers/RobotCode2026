package frc.robot.subsystems.Shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs {
        public double flywheelVelocityRPM = 0.0;
        public double flywheelAppliedVolts = 0.0;
        public double flywheelCurrentAmps = 0.0;

        public double hoodPositionRad = 0.0;
        public double hoodVelocityRadPerSec = 0.0;
        public double hoodAppliedVolts = 0.0;
        public double hoodCurrentAmps = 0.0;
        public boolean ShooterIR = false;
    }

    public default void updateInputs(ShooterIOInputs inputs){}

    public default void setVoltage(double volts){}

    default void setVelocity(double velocityRPM) {}

    default void setPID(double kP, double kI, double kD) {}

    default void setFF(double kS, double kV, double kA) {}
}