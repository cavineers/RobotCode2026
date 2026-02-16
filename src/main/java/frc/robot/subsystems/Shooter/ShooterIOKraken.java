package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import static frc.robot.subsystems.Shooter.ShooterConstants.*;

/** Dual Kraken x44 shooter implementation with FOC */
public class ShooterIOKraken implements ShooterIO {
    
    private final TalonFX leaderMotor;
    private final TalonFX followerMotor;
    private final VelocityVoltage velocityControl;
    private final VoltageOut voltageControl = new VoltageOut(0);

    public ShooterIOKraken() {
        leaderMotor = new TalonFX(kFlywheelLeaderMotorCanID, kFlywheelCanBus);
        followerMotor = new TalonFX(kFlywheelFollowerMotorCanID, kFlywheelCanBus);
        
        velocityControl = new VelocityVoltage(0)
            .withSlot(0)
            .withEnableFOC(kEnableFOC);

        var leaderConfig = new TalonFXConfiguration();
        var followerConfig = new TalonFXConfiguration();
        
        // Leader motor setup
        leaderConfig.MotorOutput.NeutralMode = kNeutralMode;
        leaderConfig.MotorOutput.Inverted = kLeaderMotorInverted;
        leaderConfig.CurrentLimits.SupplyCurrentLimit = kSupplyCurrentLimit;
        leaderConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        leaderConfig.CurrentLimits.StatorCurrentLimit = kStatorCurrentLimit;
        leaderConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        
        // PID + Feedforward (kV is V/(rot/s) at motor shaft)
        leaderConfig.Slot0.kP = kP;
        leaderConfig.Slot0.kI = kI;
        leaderConfig.Slot0.kD = kD;
        leaderConfig.Slot0.kS = kS;
        leaderConfig.Slot0.kV = kV;
        leaderConfig.Slot0.kA = kA;
        
        // Follower motor setup (no inversion needed, handled by Follower control)
        followerConfig.MotorOutput.NeutralMode = kNeutralMode;
        followerConfig.CurrentLimits.SupplyCurrentLimit = kSupplyCurrentLimit;
        followerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        followerConfig.CurrentLimits.StatorCurrentLimit = kStatorCurrentLimit;
        followerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        
        leaderMotor.getConfigurator().apply(leaderConfig);
        followerMotor.getConfigurator().apply(followerConfig);
        
        // Set follower to mirror leader output
        followerMotor.setControl(new Follower(kFlywheelLeaderMotorCanID, MotorAlignmentValue.Opposed));
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.flywheelVelocityRPM = leaderMotor.getVelocity().getValueAsDouble() * 60.0 / kGearRatio;
        inputs.flywheelAppliedVolts = leaderMotor.getMotorVoltage().getValueAsDouble();
        inputs.flywheelCurrentAmps = leaderMotor.getSupplyCurrent().getValueAsDouble();
        inputs.flywheelTempCelsius = leaderMotor.getDeviceTemp().getValueAsDouble();
        
        inputs.followerVelocityRPM = followerMotor.getVelocity().getValueAsDouble() * 60.0 / kGearRatio;
        inputs.followerAppliedVolts = followerMotor.getMotorVoltage().getValueAsDouble();
        inputs.followerCurrentAmps = followerMotor.getSupplyCurrent().getValueAsDouble();
        inputs.followerTempCelsius = followerMotor.getDeviceTemp().getValueAsDouble();
        
        inputs.connected = leaderMotor.isAlive() && followerMotor.isAlive();
    }

    @Override
    public void setVelocity(double velocityRPM) {
        double velocityRPS = (velocityRPM / 60.0) * kGearRatio;
        leaderMotor.setControl(velocityControl.withVelocity(velocityRPS));
    }

    @Override
    public void setVoltage(double volts) {
        leaderMotor.setControl(voltageControl.withOutput(volts));
    }

    @Override
    public void stop() {
        leaderMotor.stopMotor();
    }

    @Override
    public void setPID(double kP, double kI, double kD) {
        var config = new TalonFXConfiguration();
        config.Slot0.kP = kP;
        config.Slot0.kI = kI;
        config.Slot0.kD = kD;
        leaderMotor.getConfigurator().apply(config.Slot0);
    }

    @Override
    public void setFF(double kS, double kV, double kA) {
        var config = new TalonFXConfiguration();
        config.Slot0.kS = kS;
        config.Slot0.kV = kV;
        config.Slot0.kA = kA;
        leaderMotor.getConfigurator().apply(config.Slot0);
    }
}
