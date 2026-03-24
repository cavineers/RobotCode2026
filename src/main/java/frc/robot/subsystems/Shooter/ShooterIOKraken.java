package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;


import static frc.robot.subsystems.Shooter.ShooterConstants.*;

/** Dual Kraken x44 shooter implementation with FOC */
public class ShooterIOKraken implements ShooterIO {
    
    private final TalonFX leaderMotor;
    private final TalonFX followerMotor;
    private final VelocityVoltage velocityControl;
    private final VoltageOut voltageControl = new VoltageOut(0);
    private final CANBus shooterCANBus = new CANBus(kFlywheelCanBus);
    
    // Deadband management to prevent oscillations
    private double targetVelocityRPM = 0.0;
    private double lastFeedbackVelocityRPM = 0.0;

    
    // WPILib Alerts for error handling
    private final Alert leaderConfigAlert = new Alert("Shooter leader motor config failed", AlertType.kError);
    private final Alert followerConfigAlert = new Alert("Shooter follower motor config failed", AlertType.kError);
    private final Alert setPIDAlert = new Alert("Shooter setPID failed", AlertType.kWarning);
    private final Alert setFFAlert = new Alert("Shooter setFF failed", AlertType.kWarning);

    public ShooterIOKraken() {
        
        leaderMotor = new TalonFX(kFlywheelLeaderMotorCanID, shooterCANBus);
        followerMotor = new TalonFX(kFlywheelFollowerMotorCanID, shooterCANBus);
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
        
        // PID + Feedforward configuration
        // Slot 0: High P for aggressive response, clamped by motor's 12V output limits
        // Slot 1: Feedforward only (kP=0) for use in deadband phase
        leaderConfig.Slot0.kP = kP;
        leaderConfig.Slot0.kI = kI;
        leaderConfig.Slot0.kD = kD;
        leaderConfig.Slot0.kS = kS;
        leaderConfig.Slot0.kV = kV;
        leaderConfig.Slot0.kA = kA;
        
        // Slot 1: Same feedforward, but zero P (disables PID correction)
        leaderConfig.Slot1.kP = kP_FFOnly;
        leaderConfig.Slot1.kI = kI_FFOnly;
        leaderConfig.Slot1.kD = kD_FFOnly;
        leaderConfig.Slot1.kS = kS;
        leaderConfig.Slot1.kV = kV;
        leaderConfig.Slot1.kA = kA;
        
        // Follower motor setup (minimal config - just current limits)
        followerConfig.MotorOutput.NeutralMode = kNeutralMode;
        followerConfig.CurrentLimits.SupplyCurrentLimit = kSupplyCurrentLimit;
        followerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        followerConfig.CurrentLimits.StatorCurrentLimit = kStatorCurrentLimit;
        followerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        
        // Apply configurations with error checking
        StatusCode leaderStatus = leaderMotor.getConfigurator().apply(leaderConfig);
        StatusCode followerStatus = followerMotor.getConfigurator().apply(followerConfig);
        
        leaderConfigAlert.set(!leaderStatus.isOK());
        followerConfigAlert.set(!followerStatus.isOK());
        
        // Set follower to mirror leader output (opposed means it runs opposite direction)
        followerMotor.setControl(new Follower(kFlywheelLeaderMotorCanID, MotorAlignmentValue.Opposed));
        
        // Configure signal update frequencies for real-time control
        BaseStatusSignal.setUpdateFrequencyForAll(100.0, // 100Hz for velocity control
            leaderMotor.getVelocity(),
            followerMotor.getVelocity()
        );
        
        BaseStatusSignal.setUpdateFrequencyForAll(50.0, // 50Hz for telemetry
            leaderMotor.getMotorVoltage(),
            leaderMotor.getSupplyCurrent(),
            leaderMotor.getDeviceTemp(),
            followerMotor.getMotorVoltage(),
            followerMotor.getSupplyCurrent(),
            followerMotor.getDeviceTemp()
        );
        
        // Optimize CAN bus utilization by reducing unused signals
        leaderMotor.optimizeBusUtilization();
        followerMotor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        // getVelocity() returns motor shaft RPS — divide by kGearRatio (motor/flywheel) to get flywheel RPM
        inputs.flywheelVelocityRPM = leaderMotor.getVelocity().getValueAsDouble() * 60.0 / kGearRatio;
        inputs.flywheelAppliedVolts = leaderMotor.getMotorVoltage().getValueAsDouble();
        inputs.flywheelCurrentAmps = leaderMotor.getSupplyCurrent().getValueAsDouble();
        inputs.flywheelTempCelsius = leaderMotor.getDeviceTemp().getValueAsDouble();

        inputs.followerVelocityRPM = followerMotor.getVelocity().getValueAsDouble() * 60.0 / kGearRatio;
        inputs.followerAppliedVolts = followerMotor.getMotorVoltage().getValueAsDouble();
        inputs.followerCurrentAmps = followerMotor.getSupplyCurrent().getValueAsDouble();
        inputs.followerTempCelsius = followerMotor.getDeviceTemp().getValueAsDouble();

        inputs.connected = leaderMotor.isAlive() && followerMotor.isAlive();
        
        // Store feedback velocity for deadband logic
        lastFeedbackVelocityRPM = inputs.flywheelVelocityRPM;
        
        // Apply deadband logic: switch control slot based on error
        // Slot 0: High kP for aggressive response (far from target)
        // Slot 1: kP=0, feedforward only (near target, prevents oscillations)
        if (targetVelocityRPM > 0) {
            double error = Math.abs(targetVelocityRPM - lastFeedbackVelocityRPM);
            
            if (error <= kVelocityErrorDeadbandRPM) {
                // Within deadband: use Slot 1 (feedforward only, kP=0)
                double velocityRPS = (targetVelocityRPM / 60.0) * kGearRatio;
                leaderMotor.setControl(velocityControl.withVelocity(velocityRPS).withSlot(1));
            } else {
                // Outside deadband: use Slot 0 (high kP for aggressive response)
                double velocityRPS = (targetVelocityRPM / 60.0) * kGearRatio;
                leaderMotor.setControl(velocityControl.withVelocity(velocityRPS).withSlot(0));
            }
        }
    }

    @Override
    public void setVelocity(double velocityRPM) {
        // Store target for deadband logic in updateInputs
        targetVelocityRPM = velocityRPM;
        
        // Convert RPM to RPS for Phoenix 6
        double velocityRPS = (velocityRPM / 60.0) * kGearRatio;
        // Default to Slot 0 (high kP). Will switch to Slot 1 in updateInputs if within deadband.
        leaderMotor.setControl(velocityControl.withVelocity(velocityRPS).withSlot(0));
    }

    @Override
    public void setVoltage(double volts) {
        leaderMotor.setControl(voltageControl.withOutput(volts));
    }

    @Override
    public void stop() {
        targetVelocityRPM = 0.0;
        leaderMotor.stopMotor();
    }

    @Override
    public void setPID(double kP, double kI, double kD) {
        // Read current configuration, modify only PID values, then reapply
        var config = new TalonFXConfiguration();
        leaderMotor.getConfigurator().refresh(config);
        config.Slot0.kP = kP;
        config.Slot0.kI = kI;
        config.Slot0.kD = kD;
        
        StatusCode status = leaderMotor.getConfigurator().apply(config.Slot0);
        setPIDAlert.set(!status.isOK());
    }

    @Override
    public void setFF(double kS, double kV, double kA) {
        // Read current configuration, modify only FF values, then reapply
        var config = new TalonFXConfiguration();
        leaderMotor.getConfigurator().refresh(config);
        config.Slot0.kS = kS;
        config.Slot0.kV = kV;
        config.Slot0.kA = kA;
        
        StatusCode status = leaderMotor.getConfigurator().apply(config.Slot0);
        setFFAlert.set(!status.isOK());
    }
}
