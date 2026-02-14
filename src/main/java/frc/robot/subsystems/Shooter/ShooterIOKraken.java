package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import static frc.robot.subsystems.Shooter.ShooterConstants.*;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;

public class ShooterIOKraken implements ShooterIO {
    
    private final TalonFX flywheelMotor;
    private final TalonFX followerMotor;
    private final VelocityVoltage velocityControl;
    private final VoltageOut voltageControl = new VoltageOut(0);

    public ShooterIOKraken() {
        flywheelMotor = new TalonFX(kFlywheelCanID);
        followerMotor = new TalonFX(kFollowerCanID);
        
        // Enable FOC in velocity control
        velocityControl = new VelocityVoltage(0)
            .withSlot(0)
            .withEnableFOC(kEnableFOC);

        var flywheelConfig = new TalonFXConfiguration();
        var followerConfig = new TalonFXConfiguration();
        
        // Motor configuration
        flywheelConfig.MotorOutput.NeutralMode = kFlywheelNeutralMode;
        flywheelConfig.MotorOutput.Inverted = kFlywheelMotorInverted;
        followerConfig.MotorOutput.NeutralMode = kFlywheelNeutralMode;
        followerConfig.MotorOutput.Inverted = kFollowerMotorInverted;
        
        // Current limits
        flywheelConfig.CurrentLimits.SupplyCurrentLimit = kSupplyCurrentLimit;
        flywheelConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        flywheelConfig.CurrentLimits.StatorCurrentLimit = kStatorCurrentLimit;
        flywheelConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        followerConfig.CurrentLimits.SupplyCurrentLimit = kSupplyCurrentLimit;
        followerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        followerConfig.CurrentLimits.StatorCurrentLimit = kStatorCurrentLimit;
        followerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        
        // PID flywheelConfiguration (slot 0)
        flywheelConfig.Slot0.kP = kP;
        flywheelConfig.Slot0.kI = kI;
        flywheelConfig.Slot0.kD = kD;
        flywheelConfig.Slot0.kS = kS;
        flywheelConfig.Slot0.kV = kV;
        flywheelConfig.Slot0.kA = kA;
        
        flywheelMotor.getConfigurator().apply(flywheelConfig);
        followerMotor.getConfigurator().apply(followerConfig);
        followerMotor.setControl(new Follower(kFlywheelCanID, MotorAlignmentValue.Opposed));//TODO: Check
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
        var flywheelConfig = new TalonFXConfiguration();
        flywheelConfig.Slot0.kP = kP;
        flywheelConfig.Slot0.kI = kI;
        flywheelConfig.Slot0.kD = kD;
        flywheelMotor.getConfigurator().apply(flywheelConfig.Slot0);
    }

    @Override
    public void setFF(double kS, double kV, double kA) {
        var flywheelConfig = new TalonFXConfiguration();
        flywheelConfig.Slot0.kS = kS;
        flywheelConfig.Slot0.kV = kV;
        flywheelConfig.Slot0.kA = kA;
        flywheelMotor.getConfigurator().apply(flywheelConfig.Slot0);
    }

    @AutoLogOutput(key = "Shooter/PIDError")
    public double getPIDError(){
        return this.flywheelMotor.getClosedLoopError(true).getValueAsDouble();
    }
}