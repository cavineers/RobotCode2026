package frc.robot.subsystems.Shooter;

import org.littletonrobotics.junction.AutoLog;

/**
 * @brief Hardware interface for the shooter flywheel.
 */
public interface ShooterIO {
    
    @AutoLog
    public static class ShooterIOInputs {
        public double flywheelVelocityRPM = 0.0;
        public double flywheelAppliedVolts = 0.0;
        public double flywheelCurrentAmps = 0.0;
        public double flywheelTempCelsius = 0.0;
        
        // Follower motor telemetry
        public double followerVelocityRPM = 0.0;
        public double followerAppliedVolts = 0.0;
        public double followerCurrentAmps = 0.0;
        public double followerTempCelsius = 0.0;
        
        // Hood servo telemetry
        public double hoodAngleDegrees = 0.0;
        public double hoodServoPosition = 0.0; // 0.0 to 1.0
        
        public boolean connected = false;
    }

    /**
     * @brief Update inputs from hardware.
     * @param inputs Input object to populate
     */
    default void updateInputs(ShooterIOInputs inputs) {}

    /**
     * @brief Set flywheel velocity in RPM.
     * @param velocityRPM Target velocity in RPM
     */
    default void setVelocity(double velocityRPM) {}

    /**
     * @brief Set flywheel voltage (open loop).
     * @param volts Voltage to apply
     */
    default void setVoltage(double volts) {}

    /**
     * @brief Stop the flywheel.
     */
    default void stop() {}

    /**
     * @brief Set PID gains for velocity control.
     * @param kP Proportional gain
     * @param kI Integral gain
     * @param kD Derivative gain
     */
    default void setPID(double kP, double kI, double kD) {}

    /**
     * @brief Set feedforward gains for velocity control.
     * @param kS Static friction (V)
     * @param kV Velocity feedforward (V/(rad/s))
     * @param kA Acceleration feedforward (V/(rad/s^2))
     */
    default void setFF(double kS, double kV, double kA) {}

    /**
     * @brief Set hood angle in degrees.
     * @param angleDegrees Target hood angle in degrees
     */
    default void setHoodAngle(double angleDegrees) {}

    /**
     * @brief Set hood servo position directly (0.0 to 1.0).
     * @param position Servo position from 0.0 to 1.0
     */
    default void setHoodServoPosition(double position) {}
}

