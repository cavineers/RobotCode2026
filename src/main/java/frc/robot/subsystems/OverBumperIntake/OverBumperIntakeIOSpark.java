


package frc.robot.subsystems.OverBumperIntake;

import static frc.lib.SparkUtil.*;

import static frc.robot.subsystems.OverBumperIntake.OverBumperIntakeConstants.*;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class OverBumperIntakeIOSpark implements OverBumperIntakeIO {
    private final SparkMax intakeMotor = new SparkMax(kIntakeMotorCanID, MotorType.kBrushless);
    private final RelativeEncoder intakeEncoder = intakeMotor.getEncoder();

    private final SparkMax deployMotor = new SparkMax(kDeployMotorCanID, MotorType.kBrushless);
    private final RelativeEncoder deployEncoder = deployMotor.getEncoder();

    private SparkMaxConfig deployConfig;
    private SparkMaxConfig intakeConfig;

    @AutoLogOutput(key="OverBumperIntake/Deployed")
    public boolean deployed = false;

    public OverBumperIntakeIOSpark() {
        deployConfig = new SparkMaxConfig();
        deployConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(OverBumperIntakeConstants.kCurrentLimit)    
            .voltageCompensation(12); 
     
        tryUntilOk(
            deployMotor,
            5,
            () -> deployMotor.configure(deployConfig, ResetMode.kResetSafeParameters,
                    PersistMode.kPersistParameters));
        
        intakeConfig = new SparkMaxConfig();
        intakeConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(OverBumperIntakeConstants.kCurrentLimit)    
            .voltageCompensation(12);
        
        tryUntilOk(
            intakeMotor,
            5,
            () -> intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters,
                    PersistMode.kPersistParameters));

    }

    @Override
    public void updateInputs(OverBumperIntakeIOInputs inputs) {
        ifOk(intakeMotor, intakeEncoder::getPosition, (value) -> inputs.intakePositionRad = value); //only updates the value if the output is valid
        ifOk(intakeMotor, intakeEncoder::getVelocity, (value) -> inputs.intakeVelocityRadPerSec = value);
        ifOk(
            intakeMotor,
            new DoubleSupplier[] {intakeMotor::getAppliedOutput, intakeMotor::getBusVoltage},
            (values) -> inputs.intakeAppliedVolts = values[0] * values[1]);
        ifOk(intakeMotor, intakeMotor::getOutputCurrent, (value) -> inputs.intakeCurrentAmps = value);

        ifOk(deployMotor, deployEncoder::getPosition, (value) -> inputs.deployPositionRad = value); //only updates the value if the output is valid
        ifOk(deployMotor, deployEncoder::getVelocity, (value) -> inputs.deployVelocityRadPerSec = value);
        ifOk(
            deployMotor,
            new DoubleSupplier[] {deployMotor::getAppliedOutput, deployMotor::getBusVoltage},
            (values) -> inputs.deployAppliedVolts = values[0] * values[1]);
        ifOk(deployMotor, deployMotor::getOutputCurrent, (value) -> inputs.deployCurrentAmps = value);
        
        for (int i = 0; i < inputs.recentAmpsHistory.length - 1; i++) {
            inputs.recentAmpsHistory[i] = inputs.recentAmpsHistory[i + 1];
        }
        // Set the last element to currentAmps
        inputs.recentAmpsHistory[inputs.recentAmpsHistory.length - 1] = inputs.deployCurrentAmps;

        inputs.deployed = this.deployed;
    }
        
    @Override
    public void setIntakeVoltage(double volts) {
            intakeMotor.setVoltage(volts);
    }

    @Override
    public void setDeployVoltage(double volts) {
            deployMotor.setVoltage(volts);
    }

    @Override
    public void deploy() {
        setDeployVoltage(kDeployVoltage * 12.0);    
        deployed = true;
    }

    @Override
    public void retract() {
        setDeployVoltage(-kDeployVoltage * 12.0);
        deployed = false;
    }

    @Override
    public void intake() {
        setIntakeVoltage(kIntakeVoltage * 12.0);
    }

    @Override
    public void outtake() {
        setIntakeVoltage(-kIntakeVoltage * 12.0);
    }
}   