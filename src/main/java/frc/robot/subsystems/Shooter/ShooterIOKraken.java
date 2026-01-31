package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import static frc.robot.subsystems.Shooter.ShooterConstants.*;

public class ShooterIOKraken implements ShooterIO {
    
    private final TalonFX flywheelMotor;
    private final VelocityVoltage velocityControl;
    private final VoltageOut voltageControl = new VoltageOut(0);

    public ShooterIOKraken() {
        flywheelMotor = new TalonFX(kFlywheelCanID);
        
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
    }


    @Override
    public void setVoltage(double volts) {
        flywheelMotor.setControl(voltageControl.withOutput(volts));
    }

    @Override
    public void setVelocity(double velocityRPM) {
        flywheelMotor.setControl(velocityControl.withVelocity(velocityRPM / 60.0));
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