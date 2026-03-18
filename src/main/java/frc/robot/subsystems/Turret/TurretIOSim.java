package frc.robot.subsystems.Turret;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;

/**
 * Simulation-side implementation of {@link TurretIO}. Models the turret as a simple
 * rotational inertia driven by a single brushless motor with onboard PID emulation.
 */
public class TurretIOSim implements TurretIO {

	private static final double INTERNAL_STEP_SECONDS = 0.001;
	private static final DCMotor MOTOR_MODEL = DCMotor.getNEO(1).withReduction(TurretConstants.kGearRatio);

	private final PIDController positionController =
			new PIDController(2.0, TurretConstants.kPositionKi, TurretConstants.kPositionKd);

	private double positionRad = TurretConstants.kSimStartingAngleRad;
	private double velocityRadPerSec = 0.0;
	private double appliedVolts = 0.0;
	private double supplyCurrentAmps = 0.0;

	@AutoLogOutput(key = "Turret/SimTargetAngleRad")
	private double targetPositionRad = TurretConstants.kSimStartingAngleRad;

	private boolean closedLoopEnabled = false;
	private double manualVoltage = 0.0;
	private boolean brakeModeEnabled = TurretConstants.kBrakeModeEnabled;

	public TurretIOSim() {
		// positionController.enableContinuousInput(TurretConstants.kMinAngleRad, TurretConstants.kMaxAngleRad);
		positionController.setTolerance(TurretConstants.kPositionToleranceRad);
		positionController.setSetpoint(targetPositionRad);
	}

	@Override
	public void updateInputs(TurretIOInputs inputs) {
		simulateStep();

		inputs.positionRad = positionRad;
		inputs.velocityRadPerSec = velocityRadPerSec;
		inputs.appliedVolts = appliedVolts;
		inputs.supplyCurrentAmps = Math.abs(supplyCurrentAmps);
		inputs.motorTempCelsius = 40.0; // placeholder temperature
		
		inputs.forwardLimit = positionRad >= TurretConstants.kMaxAngleRad - 1e-6;
		inputs.reverseLimit = positionRad <= TurretConstants.kMinAngleRad + 1e-6;
	}

	@Override
	public void setVoltage(double volts) {
		manualVoltage = MathUtil.clamp(volts, -TurretConstants.kMaxVoltage, TurretConstants.kMaxVoltage);
		closedLoopEnabled = false;
		positionController.reset();
	}

	@Override
	public void setPositionSetpoint(double positionRad) {
		targetPositionRad = clampToRange(positionRad);
		positionController.setSetpoint(targetPositionRad);
		closedLoopEnabled = true;
		manualVoltage = 0.0;
	}

	@Override
	public void resetEncoder(double positionRad) {
		this.positionRad = clampToRange(positionRad);
		velocityRadPerSec = 0.0;
		positionController.reset();
		positionController.setSetpoint(this.positionRad);
	}

	@Override
	public void setBrakeMode(boolean enable) {
		brakeModeEnabled = enable;
	}

	@Override
	public void stop() {
		manualVoltage = 0.0;
		appliedVolts = 0.0;
		closedLoopEnabled = false;
		positionController.reset();
	}

	@Override
	public void configureClosedLoop(double kp, double ki, double kd) {
		positionController.setPID(kp, ki, kd);
	}

	private void simulateStep() {
		double commandVoltage = closedLoopEnabled
				? positionController.calculate(positionRad)
				: manualVoltage;

		commandVoltage = MathUtil.clamp(commandVoltage, -TurretConstants.kMaxVoltage, TurretConstants.kMaxVoltage);

		int substeps = Math.max(1,
				(int) Math.round(TurretConstants.kSimDtSeconds / INTERNAL_STEP_SECONDS));
		double dt = TurretConstants.kSimDtSeconds / substeps;

		for (int i = 0; i < substeps; i++) {
			applyMotorPhysics(commandVoltage, dt);
		}

		appliedVolts = commandVoltage;
	}

	private void applyMotorPhysics(double voltage, double dt) {
		double backEmf = velocityRadPerSec / MOTOR_MODEL.KvRadPerSecPerVolt;
		double current = (voltage - backEmf) / MOTOR_MODEL.rOhms;
		current = MathUtil.clamp(current, -MOTOR_MODEL.stallCurrentAmps, MOTOR_MODEL.stallCurrentAmps);

	double torque = MOTOR_MODEL.KtNMPerAmp * current;
		double damping = (brakeModeEnabled ? TurretConstants.kSimBrakeTorquePerRadPerSec
				: TurretConstants.kSimFrictionTorquePerRadPerSec) * velocityRadPerSec;
		torque -= damping;

		double angularAcceleration = torque / TurretConstants.kSimMomentOfInertia;

		velocityRadPerSec += angularAcceleration * dt;
		positionRad += velocityRadPerSec * dt;

		enforceLimits();

		supplyCurrentAmps = current;
	}

	private void enforceLimits() {
		if (positionRad <= TurretConstants.kMinAngleRad) {
			positionRad = TurretConstants.kMinAngleRad;
			if (velocityRadPerSec < 0.0) {
				velocityRadPerSec = 0.0;
			}
		} else if (positionRad >= TurretConstants.kMaxAngleRad) {
			positionRad = TurretConstants.kMaxAngleRad;
			if (velocityRadPerSec > 0.0) {
				velocityRadPerSec = 0.0;
			}
		}
	}

	private static double clampToRange(double angleRad) {
		double wrapped = MathUtil.angleModulus(angleRad); // Wraps to -PI to PI
        // After wrapping to [-PI, PI], the turret limits are enforced by clamping
        // to [TurretConstants.kMinAngleRad, TurretConstants.kMaxAngleRad].
        // This keeps the angle within the configured mechanical range.
		return MathUtil.clamp(wrapped, TurretConstants.kMinAngleRad, TurretConstants.kMaxAngleRad);
	}
}
