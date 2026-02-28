package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

import static frc.robot.subsystems.Shooter.ShooterConstants.*;

/**
 * @brief Shooter IO simulation implementation.
 */
public class ShooterIOSim implements ShooterIO {
    
    private final DCMotorSim flywheelSim;
    private double appliedVolts = 0.0;
    private double currentHoodAngleDegrees = 0.0;
    private double currentServoPosition = 0.0;

    public ShooterIOSim() {
        // Note: FOC is handled in hardware, simulation uses standard model
        flywheelSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX44(2), 
                kFlywheelMOI, 
                kGearRatio
            ),
            DCMotor.getKrakenX44(2)
        );
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        flywheelSim.update(0.02); // 20ms loop time
        
        inputs.flywheelVelocityRPM = flywheelSim.getAngularVelocityRPM();
        inputs.flywheelAppliedVolts = appliedVolts;
        inputs.flywheelCurrentAmps = flywheelSim.getCurrentDrawAmps();
        inputs.flywheelTempCelsius = 25.0; // Simulated temp
        
        // Follower mirrors leader in sim
        inputs.followerVelocityRPM = inputs.flywheelVelocityRPM;
        inputs.followerAppliedVolts = appliedVolts;
        inputs.followerCurrentAmps = flywheelSim.getCurrentDrawAmps();
        inputs.followerTempCelsius = 25.0;
        
        inputs.hoodAngleDegrees = currentHoodAngleDegrees;
        inputs.hoodServoPosition = currentServoPosition;
        
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

    @Override
    public void setHoodAngle(double angleDegrees) {
        // Clamp angle to valid range
        currentHoodAngleDegrees = MathUtil.clamp(angleDegrees, kMinHoodAngleDegrees, kMaxHoodAngleDegrees);
        
        // Map angle to servo position (linear interpolation)
        currentServoPosition = (currentHoodAngleDegrees - kMinHoodAngleDegrees) / 
                               (kMaxHoodAngleDegrees - kMinHoodAngleDegrees);
        currentServoPosition = kMinServoPosition + currentServoPosition * (kMaxServoPosition - kMinServoPosition);
    }

    @Override
    public void setHoodServoPosition(double position) {
        currentServoPosition = MathUtil.clamp(position, kMinServoPosition, kMaxServoPosition);
        
        // Update angle based on position
        double normalizedPosition = (currentServoPosition - kMinServoPosition) / 
                                    (kMaxServoPosition - kMinServoPosition);
        currentHoodAngleDegrees = kMinHoodAngleDegrees + 
                                  normalizedPosition * (kMaxHoodAngleDegrees - kMinHoodAngleDegrees);
    }
}

