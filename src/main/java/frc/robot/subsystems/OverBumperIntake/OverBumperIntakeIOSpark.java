package frc.robot.subsystems.OverBumperIntake;

import static frc.lib.SparkUtil.*; //has a bunch of untility functions for SparkMax

import static frc.robot.subsystems.OverBumperIntake.OverBumperIntakeConstants.*;

import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

public class OverBumperIntakeIOSpark implements OverBumperIntakeIO {
    private final SparkMax intakeMotor = new SparkMax(kIntakeMotorCanID, MotorType.kBrushless);
    private final RelativeEncoder intakeEncoder = intakeMotor.getEncoder();

    private final SparkMax deployMotor = new SparkMax(kDeployMotorCanID, MotorType.kBrushless);
    private final RelativeEncoder deployEncoder = deployMotor.getEncoder();

    public OverBumperIntakeIOSpark() {
        //Could do motor configuration here
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
    }
        
    @Override
    public void setIntakeVoltage(double volts) {
            intakeMotor.setVoltage(volts);
    }

    @Override
    public void setDeployVoltage(double volts) {
            deployMotor.setVoltage(volts);
    }
}

