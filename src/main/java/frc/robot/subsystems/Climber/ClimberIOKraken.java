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
import edu.wpi.first.wpilibj.DigitalInput;

public class ClimberIOKraken implements ClimberIO {
    @AutoLogOutput(key="Climber/Setpoint")
    private double absSetpoint = 0;

    @AutoLogOutput(key="Climber/IsClosed")
    private boolean isClosed = false;
    
    public enum ClimbState{
        RESTING,
        DEPLOYED,
        ENGAGED
    }
    
    @AutoLogOutput(key="Climber/ClimbState")
    private ClimbState climbState = ClimbState.RESTING;

    private DigitalInput limitSwitch = new DigitalInput(kLimitSwitchID);
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
        
        Logger.recordOutput("Climber/climberPositionRotations", inputs.climberPositionRotations);
        Logger.recordOutput("Climber/limitSwitchPressed", this.limitSwitchPressed());
        if (this.isClosed){
            climberMotor.setControl(m_request.withPosition(absSetpoint));
        }

        if (this.limitSwitchPressed()){
            climbState = ClimbState.RESTING;
            climberMotor.setPosition(kRestMotorRotations);
        }

    }

    @Override
    public void updateClimberSetpoint(double setpoint) {
        this.absSetpoint = this.clipSetpoint(setpoint);
        this.setClosedLoop(true);
    }

    public double clipSetpoint(double setpoint) {
        if (absSetpoint > ClimberConstants.kDeployedMotorRotations) {
            return ClimberConstants.kDeployedMotorRotations;
        }
        else if (absSetpoint < ClimberConstants.kRestMotorRotations) {
             return ClimberConstants.kRestMotorRotations;
        }
        return setpoint;
    }

    @Override
    public void setClosedLoop(boolean val) {
        this.isClosed = val;
    }

    @Override
    public void deploy() {
        updateClimberSetpoint(kDeployedMotorRotations);
    }

    @Override
    public void retract() {
        updateClimberSetpoint(kRestMotorRotations);
    }

    @Override
    public void engage() {
        updateClimberSetpoint(kEngagedMotorRotations);
    }

    public boolean limitSwitchPressed() {
        return limitSwitch.get();
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
