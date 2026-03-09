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

import frc.robot.subsystems.InBumperIntake.InBumperIntakeIO.InBumperIntakeIOInputs;

import static frc.robot.subsystems.InBumperIntake.InBumperIntakeConstants.*;

public class InBumperIntakeIOSpark implements InBumperIntakeIO {
    private final SparkMax bottomMotor = new SparkMax(kBottomMotorCanID, MotorType.kBrushless);
    private final RelativeEncoder bottomEncoder = bottomMotor.getEncoder();

    private final SparkMax topMotor = new SparkMax(kTopMotorCanID, MotorType.kBrushless);
    private final RelativeEncoder topEncoder = topMotor.getEncoder();

    private final SparkMax outsideMotor = new SparkMax(kOutsideMotorCanID, MotorType.kBrushless); 
    private final RelativeEncoder outsideEncoder = outsideMotor.getEncoder();

    private final SparkFlex spindexerMotor = new SparkFlex(kSpindexerMotorCanID, MotorType.kBrushless);
    private final RelativeEncoder spindexerEncoder = spindexerMotor.getEncoder();

    private SparkMaxConfig bottomConfig;
    private SparkMaxConfig topConfig;
    private SparkMaxConfig outsideConfig;
    private SparkFlexConfig spindexerConfig;

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

        spindexerConfig = new SparkFlexConfig();
        spindexerConfig
            .idleMode(kSpindexerIdleMode)
            .smartCurrentLimit(kSpindexerCurrentLimit)
            .voltageCompensation(12)
            .inverted(kSpindexerInverted);
        tryUntilOk(
            spindexerMotor,
            5,
            () -> spindexerMotor.configure(spindexerConfig, ResetMode.kResetSafeParameters,
                    PersistMode.kPersistParameters));
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

        ifOk(spindexerMotor, spindexerEncoder::getPosition, (value) -> inputs.spindexerMotorPositionRad = value);
        ifOk(spindexerMotor, spindexerEncoder::getVelocity, (value) -> inputs.spindexerMotorVelocityRadPerSec = value);
        ifOk(
            spindexerMotor,
            new DoubleSupplier[] {spindexerMotor::getAppliedOutput, spindexerMotor::getBusVoltage},
            (values) -> inputs.spindexerMotorAppliedVolts = values[0] * values[1]);
        ifOk(spindexerMotor, spindexerMotor::getOutputCurrent, (value) -> inputs.spindexerMotorCurrentAmps = value);
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
    public void setSpindexerVoltage(double volts) {
        spindexerMotor.setVoltage(volts);
    }

}