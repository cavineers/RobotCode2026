package frc.robot.subsystems.Turret;

import org.littletonrobotics.junction.AutoLog;

/**
 * Hardware abstraction for the turret mechanism. Implementations provide the
 * bridge between hardware (real or simulated) and the {@link TurretSubsystem}.
 */
public interface TurretIO {

	@AutoLog
	class TurretIOInputs {
		public double positionRad = 0.0;
		public double velocityRadPerSec = 0.0;
		public double appliedVolts = 0.0;
		public double supplyCurrentAmps = 0.0;
		public double motorTempCelsius = 0.0;
		public boolean forwardLimit = false;
		public boolean reverseLimit = false;
		/** True when the homing limit switch is triggered (already accounts for normally-open/closed). */
		public boolean homeSwitchTriggered = false;
	}

	default void updateInputs(TurretIOInputs inputs) {}

	default void setVoltage(double volts) {}

	default void setPositionSetpoint(double positionRad) {}

	default void resetEncoder(double positionRad) {}

	default void setBrakeMode(boolean enable) {}

	default void stop() {}

	default void configureClosedLoop(double kp, double ki, double kd) {}
}
