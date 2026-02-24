package frc.robot.subsystems.Climber;

import static frc.robot.subsystems.Climber.ClimberConstants.*;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

public class ClimberIOKraken implements ClimberIO {
    @AutoLogOutput(key="Climber/Setpoint")
    public double absSetpoint = 0;

    @AutoLogOutput(key="Climber/IsClosed")
    private boolean isClosed = false;

    private boolean tempCutoff = false;

    private final TalonFX climberMotor;
    private final VelocityVoltage velocityControl;
    private final VoltageOut voltageControl = new VoltageOut(0);
    private final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);
    
    // WPILib Alerts for error handling
    private final Alert climberConfigAlert = new Alert("Climber motor config failed", AlertType.kError);
    private final Alert setPIDAlert = new Alert("Climber setPID failed", AlertType.kWarning);

    public ClimberIOKraken() {
        climberMotor = new TalonFX(kClimberCanID);
        
        velocityControl = new VelocityVoltage(0)
            .withSlot(0)
            .withEnableFOC(kEnableFOC);

        var climberConfig = new TalonFXConfiguration();
        
        // Leader motor setup
        climberConfig.MotorOutput.NeutralMode = kClimberNeutralMode;
        climberConfig.MotorOutput.Inverted = kClimberMotorInverted;
        climberConfig.CurrentLimits.SupplyCurrentLimit = kSupplyCurrentLimit;
        climberConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        climberConfig.CurrentLimits.StatorCurrentLimit = kStatorCurrentLimit;
        climberConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        
        // PID
        climberConfig.Slot0.kP = kP;
        climberConfig.Slot0.kI = kI;
        climberConfig.Slot0.kD = kD;
                
        // Apply configurations with error checking
        StatusCode climberStatus = climberMotor.getConfigurator().apply(climberConfig);
        
        climberConfigAlert.set(!climberStatus.isOK());
        
        // Configure signal update frequencies for real-time control
        BaseStatusSignal.setUpdateFrequencyForAll(100.0, // 100Hz for velocity control
            climberMotor.getVelocity()
        );
        
        BaseStatusSignal.setUpdateFrequencyForAll(50.0, // 50Hz for telemetry
            climberMotor.getMotorVoltage(),
            climberMotor.getSupplyCurrent(),
            climberMotor.getDeviceTemp()
        );
        
        // Optimize CAN bus utilization by reducing unused signals
        climberMotor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        inputs.climberVelocityRotationsPerSec = climberMotor.getVelocity().getValueAsDouble();
        inputs.climberAppliedVoltage = climberMotor.getMotorVoltage().getValueAsDouble();
        inputs.climberCurrentAmps = climberMotor.getSupplyCurrent().getValueAsDouble();
        inputs.climberPositionRotations = climberMotor.getPosition().getValueAsDouble();
        inputs.cutoff = this.tempCutoff;
        inputs.setpoint = this.absSetpoint;
        
        Logger.recordOutput("Climber/climberPositionRotations", inputs.climberPositionRotations);
        if (this.isClosed){
            climberMotor.setControl(m_request.withPosition(absSetpoint));
        }
        for (int i = 0; i < inputs.recentAmpsHistory.length - 1; i++) {
            inputs.recentAmpsHistory[i] = inputs.recentAmpsHistory[i + 1];
        }
        // Set the last element to currentAmps
        inputs.recentAmpsHistory[inputs.recentAmpsHistory.length - 1] = inputs.climberCurrentAmps;

        double sum = 0;
        for (double value : inputs.recentAmpsHistory) {
            sum += value;
        }
        Logger.recordOutput("OverBumperIntake/AverageAmps", sum / inputs.recentAmpsHistory.length);
        if (sum / inputs.recentAmpsHistory.length > kCutOffAmps) {
            tempCutoff = true;
        } else {
            tempCutoff = false;
        }
    }

    @Override
    public void resetEncoder(double rotations) {
        climberMotor.setPosition(rotations);
    }

    @Override
    public void updateClimberSetpoint(double setpoint) {
        this.absSetpoint = this.clipSetpoint(setpoint);
        this.setClosedLoop(true);
    }

    public double clipSetpoint(double setpoint) {
        if (setpoint > ClimberConstants.kDeployedMotorRotations) {
            return ClimberConstants.kDeployedMotorRotations;
        }
        else if (setpoint < ClimberConstants.kRestMotorRotations) {
             return ClimberConstants.kRestMotorRotations;
        }
        return setpoint;
    }

    @Override
    public void setClosedLoop(boolean val) {
        this.isClosed = val;
    }

    @Override
    public void setClimberVoltage(double volts) {
        this.setClosedLoop(false);
        climberMotor.setControl(voltageControl.withOutput(volts));
    }

    @Override
    public void setPID(double kP, double kI, double kD) {
        var climberConfig = new TalonFXConfiguration();
        climberConfig.Slot0.kP = kP;
        climberConfig.Slot0.kI = kI;
        climberConfig.Slot0.kD = kD;
        climberMotor.getConfigurator().apply(climberConfig.Slot0);
    }
}
