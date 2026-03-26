package frc.robot.subsystems.InBumperIntake;

import static frc.lib.SparkUtil.*;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkFlexConfig;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.VoltageOut;

import frc.robot.subsystems.InBumperIntake.InBumperIntakeIO.InBumperIntakeIOInputs;

import static frc.robot.subsystems.InBumperIntake.InBumperIntakeConstants.*;

public class InBumperIntakeIOSpark implements InBumperIntakeIO {
    private final SparkMax bottomMotor = new SparkMax(kBottomMotorCanID, MotorType.kBrushless);
    private final RelativeEncoder bottomEncoder = bottomMotor.getEncoder();

    private final SparkMax topMotor = new SparkMax(kTopMotorCanID, MotorType.kBrushless);
    private final RelativeEncoder topEncoder = topMotor.getEncoder();

    private final SparkMax outsideMotor = new SparkMax(kOutsideMotorCanID, MotorType.kBrushless); 
    private final RelativeEncoder outsideEncoder = outsideMotor.getEncoder();

    private final TalonFX hopperMotor = new TalonFX(kHopperMotorCanID);

    private SparkMaxConfig bottomConfig;
    private SparkMaxConfig topConfig;
    private SparkMaxConfig outsideConfig;
    private TalonFXConfiguration hopperConfig;
    private VoltageOut hopperVoltageControl = new VoltageOut(0);

    public InBumperIntakeIOSpark()  {
        bottomConfig = new SparkMaxConfig();
        bottomConfig
            .idleMode(kBottomIdleMode)
            .smartCurrentLimit(kBottomCurrentLimit)
            .voltageCompensation(12)
            .inverted(kBottomInverted);
        tryUntilOk(
            bottomMotor,
            5,
            () -> bottomMotor.configure(bottomConfig, ResetMode.kResetSafeParameters,
                    PersistMode.kPersistParameters));

        topConfig = new SparkMaxConfig();
        topConfig
            .idleMode(kTopIdleMode)
            .smartCurrentLimit(kTopCurrentLimit)
            .voltageCompensation(12)
            .inverted(kTopInverted);
        tryUntilOk(
            topMotor,
            5,
            () -> topMotor.configure(topConfig, ResetMode.kResetSafeParameters,
                    PersistMode.kPersistParameters));

        outsideConfig = new SparkMaxConfig();
        outsideConfig
            .idleMode(kOutsideIdleMode)
            .smartCurrentLimit(kOutsideCurrentLimit)
            .voltageCompensation(12)
            .inverted(kOutsideInverted);
        tryUntilOk(
            outsideMotor,
            5,
            () -> outsideMotor.configure(outsideConfig, ResetMode.kResetSafeParameters,
                    PersistMode.kPersistParameters));

        hopperConfig = new TalonFXConfiguration();
        hopperConfig.MotorOutput.NeutralMode = kHopperIdleMode == IdleMode.kBrake ? 
            NeutralModeValue.Brake : NeutralModeValue.Coast;
        hopperConfig.MotorOutput.Inverted = kHopperInverted ? 
            com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive : 
            com.ctre.phoenix6.signals.InvertedValue.CounterClockwise_Positive;
        hopperConfig.CurrentLimits.SupplyCurrentLimit = kHopperCurrentLimit;
        hopperConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        hopperConfig.Voltage.PeakForwardVoltage = 12;
        hopperConfig.Voltage.PeakReverseVoltage = -12;
        hopperMotor.getConfigurator().apply(hopperConfig);
    }

    @Override
    public void updateInputs(InBumperIntakeIOInputs inputs) {
        ifOk(bottomMotor, bottomEncoder::getPosition, (value) -> inputs.bottomMotorPositionRad = value);
        ifOk(bottomMotor, bottomEncoder::getVelocity, (value) -> inputs.bottomMotorVelocityRadPerSec = value);
        ifOk(
            bottomMotor,
            new DoubleSupplier[] {bottomMotor::getAppliedOutput, bottomMotor::getBusVoltage},
            (values) -> inputs.bottomMotorAppliedVolts = values[0] * values[1]);
        ifOk(bottomMotor, bottomMotor::getOutputCurrent, (value) -> inputs.bottomMotorCurrentAmps = value);


        ifOk(topMotor, topEncoder::getPosition, (value) -> inputs.topMotorPositionRad = value);
        ifOk(topMotor, topEncoder::getVelocity, (value) -> inputs.topMotorVelocityRadPerSec = value);
        ifOk(
            topMotor,
            new DoubleSupplier[] {topMotor::getAppliedOutput, topMotor::getBusVoltage},
            (values) -> inputs.topMotorAppliedVolts = values[0] * values[1]);
        ifOk(topMotor, topMotor::getOutputCurrent, (value) -> inputs.topMotorCurrentAmps = value);


        ifOk(outsideMotor, outsideEncoder::getPosition, (value) -> inputs.outsideMotorPositionRad = value);
        ifOk(outsideMotor, outsideEncoder::getVelocity, (value) -> inputs.outsideMotorVelocityRadPerSec = value);
        ifOk(
            outsideMotor,
            new DoubleSupplier[] {outsideMotor::getAppliedOutput, outsideMotor::getBusVoltage},
            (values) -> inputs.outsideMotorAppliedVolts = values[0] * values[1]);
        ifOk(outsideMotor, outsideMotor::getOutputCurrent, (value) -> inputs.outsideMotorCurrentAmps = value);

        // Hopper (Kraken X60 via TalonFX)
        inputs.hopperMotorPositionRad = hopperMotor.getPosition().getValueAsDouble();
        inputs.hopperMotorVelocityRadPerSec = hopperMotor.getVelocity().getValueAsDouble();
        inputs.hopperMotorAppliedVolts = hopperMotor.getMotorVoltage().getValueAsDouble();
        inputs.hopperMotorCurrentAmps = hopperMotor.getSupplyCurrent().getValueAsDouble();
    }

    @Override
    public void setBottomVoltage(double volts) {
        bottomMotor.setVoltage(volts);
    }
    
    @Override
    public void setTopVoltage(double volts) {
        topMotor.setVoltage(volts);
    }

    @Override
    public void setOutsideVoltage(double volts) {
        outsideMotor.setVoltage(volts);
    }

    @Override
    public void setHopperVoltage(double volts) {
        hopperMotor.setControl(hopperVoltageControl.withOutput(volts));
    }

}