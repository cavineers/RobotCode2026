package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

import static frc.robot.subsystems.Shooter.ShooterConstants.*;

/**
 * @brief Shooter IO simulation implementation.
 */
public class ShooterIOSim implements ShooterIO {
    
    private final DCMotorSim flywheelSim;
    private double appliedVolts = 0.0;

    public ShooterIOSim() {
        flywheelSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), kFlywheelMOI, kGearRatio),
            DCMotor.getKrakenX60(1)
        );
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        flywheelSim.update(0.02); // 20ms loop time
        
        inputs.flywheelVelocityRPM = flywheelSim.getAngularVelocityRPM();
        inputs.flywheelAppliedVolts = appliedVolts;
        inputs.flywheelCurrentAmps = flywheelSim.getCurrentDrawAmps();
        inputs.flywheelTempCelsius = 25.0; // Simulated temp
        inputs.connected = true;
    }

    @Override
    public void setVelocity(double velocityRPM) {
        // Simple P controller for sim
        double error = velocityRPM - flywheelSim.getAngularVelocityRPM();
        appliedVolts = error * kSimP;
        appliedVolts = Math.max(-12.0, Math.min(12.0, appliedVolts)); // Clamp
        flywheelSim.setInputVoltage(appliedVolts);
    }

    @Override
    public void setVoltage(double volts) {
        appliedVolts = volts;
        flywheelSim.setInputVoltage(volts);
    }

    @Override
    public void stop() {
        appliedVolts = 0.0;
        flywheelSim.setInputVoltage(0.0);
    }

    @Override
    public void setPID(double kP, double kI, double kD) {
        // PID not used in sim (simple P controller above)
    }

    @Override
    public void setFF(double kS, double kV, double kA) {
        // FF not used in sim
    }
}
