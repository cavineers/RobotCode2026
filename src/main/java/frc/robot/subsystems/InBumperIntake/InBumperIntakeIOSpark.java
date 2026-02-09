package frc.robot.subsystems.InBumperIntake;

import static frc.lib.SparkUtil.*;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import static frc.robot.subsystems.InBumperIntake.InBumperIntakeConstants.*;

public class InBumperIntakeIOSpark implements InBumperIntakeIO {
    private final SparkMax bottomMotor = new SparkMax(kBottomMotorCanID, MotorType.kBrushless);
    private final RelativeEncoder bottomEncoder = bottomMotor.getEncoder();

    private final SparkMax hopperMotor = new SparkMax(kHopperMotorCanID, MotorType.kBrushless);
    private final RelativeEncoder hopperEncoder = hopperMotor.getEncoder();

    private final SparkMax topMotor = new SparkMax(kTopMotorCanID, MotorType.kBrushless); 
    private final RelativeEncoder topEncoder = topMotor.getEncoder();

    private SparkMaxConfig bottomConfig;
    private SparkMaxConfig hopperConfig;
    private SparkMaxConfig topConfig;

    public InBumperIntakeIOSpark() {
        bottomConfig = new SparkMaxConfig();
        bottomConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(InBumperIntakeConstants.kCurrentLimit)
            .voltageCompensation(12);
        tryUntilOk(
            bottomMotor,
            5,
            () -> bottomMotor.configure(bottomConfig, ResetMode.kResetSafeParameters,
                    PersistMode.kPersistParameters));

        hopperConfig = new SparkMaxConfig();
        hopperConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(InBumperIntakeConstants.kCurrentLimit)
            .voltageCompensation(12);
        tryUntilOk(
            hopperMotor,
            5,
            () -> hopperMotor.configure(hopperConfig, ResetMode.kResetSafeParameters,
                    PersistMode.kPersistParameters));

        topConfig = new SparkMaxConfig();
        topConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(InBumperIntakeConstants.kCurrentLimit)
            .voltageCompensation(12);
        tryUntilOk(
            topMotor,
            5,
            () -> topMotor.configure(topConfig, ResetMode.kResetSafeParameters,
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


        ifOk(hopperMotor, hopperEncoder::getPosition, (value) -> inputs.hopperMotorPositionRad = value);
        ifOk(hopperMotor, hopperEncoder::getVelocity, (value) -> inputs.hopperMotorVelocityRadPerSec = value);
        ifOk(
            hopperMotor,
            new DoubleSupplier[] {hopperMotor::getAppliedOutput, hopperMotor::getBusVoltage},
            (values) -> inputs.hopperMotorAppliedVolts = values[0] * values[1]);
        ifOk(hopperMotor, hopperMotor::getOutputCurrent, (value) -> inputs.hopperMotorCurrentAmps = value);


        ifOk(topMotor, topEncoder::getPosition, (value) -> inputs.topMotorPositionRad = value);
        ifOk(topMotor, topEncoder::getVelocity, (value) -> inputs.topMotorVelocityRadPerSec = value);
        ifOk(
            topMotor,
            new DoubleSupplier[] {topMotor::getAppliedOutput, topMotor::getBusVoltage},
            (values) -> inputs.topMotorAppliedVolts = values[0] * values[1]);
        ifOk(topMotor, topMotor::getOutputCurrent, (value) -> inputs.topMotorCurrentAmps = value);
    }

    @Override
    public void setBottomVoltage(double volts) {
        bottomMotor.setVoltage(volts);
    }
    
    @Override
    public void setHopperVoltage(double volts) {
        hopperMotor.setVoltage(volts);
    }

    @Override
    public void setTopVoltage(double volts) {
        topMotor.setVoltage(volts);
    }
}