package frc.robot.subsystems.InBumperIntake;

import static frc.robot.subsystems.InBumperIntake.InBumperIntakeConstants.kEnableFOC;
import static frc.robot.subsystems.InBumperIntake.InBumperIntakeConstants.kNeutralMode;
import static frc.robot.subsystems.InBumperIntake.InBumperIntakeConstants.kRollerCanBus;
import static frc.robot.subsystems.InBumperIntake.InBumperIntakeConstants.kRollerCanID;
import static frc.robot.subsystems.InBumperIntake.InBumperIntakeConstants.kRollerInverted;
import static frc.robot.subsystems.InBumperIntake.InBumperIntakeConstants.kStatorCurrentLimit;
import static frc.robot.subsystems.InBumperIntake.InBumperIntakeConstants.kSupplyCurrentLimit;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

/** Dual Kraken x44 shooter implementation with FOC */
public class InBumperIntakeIOKraken implements InBumperIntakeIO {
    
    private final TalonFX rollerMotor;
    private final VelocityVoltage velocityControl;
    private final VoltageOut voltageControl = new VoltageOut(0);
    private final CANBus rollerCANBus = new CANBus(kRollerCanBus);

    // WPILib Alerts for error handling
    private final Alert rollerConfigAlert = new Alert("Roller motor config failed", AlertType.kError);
    //private final Alert setPIDAlert = new Alert("Shooter setPID failed", AlertType.kWarning);
    //private final Alert setFFAlert = new Alert("Shooter setFF failed", AlertType.kWarning);

    public InBumperIntakeIOKraken() {
        rollerMotor = new TalonFX(kRollerCanID, rollerCANBus);
        velocityControl = new VelocityVoltage(0)
            .withSlot(0)
            .withEnableFOC(kEnableFOC);

        var rollerConfig = new TalonFXConfiguration();
        
        // Roller motor setup
        rollerConfig.MotorOutput.NeutralMode = kNeutralMode;
        rollerConfig.MotorOutput.Inverted = kRollerInverted;
        rollerConfig.CurrentLimits.SupplyCurrentLimit = kSupplyCurrentLimit;
        rollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        rollerConfig.CurrentLimits.StatorCurrentLimit = kStatorCurrentLimit;
        rollerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        
        // PID + Feedforward (kV is V/(rot/s) at motor shaft)
        // rollerConfig.Slot0.kP = kP;
        // rollerConfig.Slot0.kI = kI;
        // rollerConfig.Slot0.kD = kD;
        // rollerConfig.Slot0.kS = kS;
        // rollerConfig.Slot0.kV = kV;
        // rollerConfig.Slot0.kA = kA;

        // Apply configurations with error checking
        StatusCode rollerStatus = rollerMotor.getConfigurator().apply(rollerConfig);
        
        rollerConfigAlert.set(!rollerStatus.isOK());
        
        // Configure signal update frequencies for real-time control
        BaseStatusSignal.setUpdateFrequencyForAll(100.0, // 100Hz for velocity control
            rollerMotor.getVelocity()
        );
        
        BaseStatusSignal.setUpdateFrequencyForAll(50.0, // 50Hz for telemetry
            rollerMotor.getMotorVoltage(),
            rollerMotor.getSupplyCurrent(),
            rollerMotor.getDeviceTemp()
        );
        
        // Optimize CAN bus utilization by reducing unused signals
        rollerMotor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(InBumperIntakeIOInputs inputs) {
        inputs.rollerMotorPositionRotations = rollerMotor.getPosition().getValueAsDouble();
        inputs.rollerMotorVelocityRPM = rollerMotor.getVelocity().getValueAsDouble() * 60.0;
        inputs.rollerMotorAppliedVolts = rollerMotor.getMotorVoltage().getValueAsDouble();
        inputs.rollerMotorCurrentAmps = rollerMotor.getSupplyCurrent().getValueAsDouble();

        inputs.rollerConnected = rollerMotor.isAlive();
    }

    @Override
    public void setRollerVoltage(double volts) {
        rollerMotor.setControl(voltageControl.withOutput(volts));
    }

    /*
    @Override
    public void setVelocity(double velocityRPM) {
        // velocityRPM is FLYWHEEL RPM — multiply by kGearRatio (motor/roller) to get motor shaft RPS for Phoenix 6
        double velocityRPS = (velocityRPM / 60.0);
        rollerMotor.setControl(velocityControl.withVelocity(velocityRPS));
    }
    
    @Override
    public void setPID(double kP, double kI, double kD) {
        // Read current configuration, modify only PID values, then reapply
        var config = new TalonFXConfiguration();
        rollerMotor.getConfigurator().refresh(config);
        config.Slot0.kP = kP;
        config.Slot0.kI = kI;
        config.Slot0.kD = kD;
        
        StatusCode status = rollerMotor.getConfigurator().apply(config.Slot0);
        setPIDAlert.set(!status.isOK());
    }

    @Override
    public void setFF(double kS, double kV, double kA) {
        // Read current configuration, modify only FF values, then reapply
        var config = new TalonFXConfiguration();
        rollerMotor.getConfigurator().refresh(config);
        config.Slot0.kS = kS;
        config.Slot0.kV = kV;
        config.Slot0.kA = kA;
        
        StatusCode status = rollerMotor.getConfigurator().apply(config.Slot0);
        setFFAlert.set(!status.isOK());
    }*/
}
