package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import static frc.robot.subsystems.Shooter.ShooterConstants.*;

/**
 * @brief Shooter IO implementation using Kraken X60 (TalonFX) with FOC.
 */
public class ShooterIOKraken implements ShooterIO {
    
    private final TalonFX flywheelMotor;
    private final VelocityVoltage velocityControl;
    private final VoltageOut voltageControl = new VoltageOut(0);

    /**
     * @brief Create a Kraken X60 shooter IO.
     */
    public ShooterIOKraken() {
        flywheelMotor = new TalonFX(kFlywheelMotorCanID, kFlywheelCanBus);
        
        // Enable FOC in velocity control
        velocityControl = new VelocityVoltage(0)
            .withSlot(0)
            .withEnableFOC(kEnableFOC);

        var config = new TalonFXConfiguration();
        
        // Motor configuration
        config.MotorOutput.NeutralMode = kNeutralMode;
        config.MotorOutput.Inverted = kMotorInverted;
        
        // Current limits
        config.CurrentLimits.SupplyCurrentLimit = kSupplyCurrentLimit;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = kStatorCurrentLimit;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        
        // PID configuration (slot 0)
        config.Slot0.kP = kP;
        config.Slot0.kI = kI;
        config.Slot0.kD = kD;
        config.Slot0.kS = kS;
        config.Slot0.kV = kV;
        config.Slot0.kA = kA;
        
        flywheelMotor.getConfigurator().apply(config);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.flywheelVelocityRPM = flywheelMotor.getVelocity().getValueAsDouble() * 60.0 / kGearRatio;
        inputs.flywheelAppliedVolts = flywheelMotor.getMotorVoltage().getValueAsDouble();
        inputs.flywheelCurrentAmps = flywheelMotor.getSupplyCurrent().getValueAsDouble();
        inputs.flywheelTempCelsius = flywheelMotor.getDeviceTemp().getValueAsDouble();
        inputs.connected = flywheelMotor.isAlive();
    }

    @Override
    public void setVelocity(double velocityRPM) {
        // Convert RPM to rotations per second for Kraken
        double velocityRPS = (velocityRPM / 60.0) * kGearRatio;
        flywheelMotor.setControl(velocityControl.withVelocity(velocityRPS));
    }

    @Override
    public void setVoltage(double volts) {
        flywheelMotor.setControl(voltageControl.withOutput(volts));
    }

    @Override
    public void stop() {
        flywheelMotor.stopMotor();
    }

    @Override
    public void setPID(double kP, double kI, double kD) {
        var config = new TalonFXConfiguration();
        config.Slot0.kP = kP;
        config.Slot0.kI = kI;
        config.Slot0.kD = kD;
        flywheelMotor.getConfigurator().apply(config.Slot0);
    }

    @Override
    public void setFF(double kS, double kV, double kA) {
        var config = new TalonFXConfiguration();
        config.Slot0.kS = kS;
        config.Slot0.kV = kV;
        config.Slot0.kA = kA;
        flywheelMotor.getConfigurator().apply(config.Slot0);
    }
}
