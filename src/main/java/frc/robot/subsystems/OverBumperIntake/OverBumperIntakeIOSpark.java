


package frc.robot.subsystems.OverBumperIntake;

import static frc.lib.SparkUtil.ifOk;
import static frc.lib.SparkUtil.tryUntilOk;
import static frc.robot.subsystems.OverBumperIntake.OverBumperIntakeConstants.*;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
public class OverBumperIntakeIOSpark implements OverBumperIntakeIO {
    private final SparkFlex intakeMotor = new SparkFlex(kIntakeMotorCanID, MotorType.kBrushless);
    private final RelativeEncoder intakeEncoder = intakeMotor.getEncoder();

    private final SparkMax deployMotor = new SparkMax(kDeployMotorCanID, MotorType.kBrushless);
    private final RelativeEncoder deployEncoder = deployMotor.getEncoder();

    @AutoLogOutput(key="OverBumperIntake/motorSetpoint")
    private double motorSetpoint = 0;

    private SparkMaxConfig deployConfig;
    private SparkFlexConfig intakeConfig;

    PIDController controller = new PIDController(kProportionalGainSpark, kIntegralTermSpark, kDerivativeTermSpark);

    public boolean isClosed = false;

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
        
        intakeConfig = new SparkFlexConfig();
        intakeConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(60)
            .inverted(true)
            .voltageCompensation(12);
            
        
        tryUntilOk(
            intakeMotor,
            5,
            () -> intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters,
                    PersistMode.kPersistParameters));

    }

    @Override
    public void updateInputs(OverBumperIntakeIOInputs inputs) {
        ifOk(intakeMotor, intakeEncoder::getPosition, (value) -> inputs.intakePositionRotations = value); //only updates the value if the output is valid
        ifOk(intakeMotor, intakeEncoder::getVelocity, (value) -> inputs.intakeVelocityRotationsPerSec = value);
        ifOk(
            intakeMotor,
            new DoubleSupplier[] {intakeMotor::getAppliedOutput, intakeMotor::getBusVoltage},
            (values) -> inputs.intakeAppliedVolts = values[0] * values[1]);
        ifOk(intakeMotor, intakeMotor::getOutputCurrent, (value) -> inputs.intakeCurrentAmps = value);

        ifOk(deployMotor, deployEncoder::getPosition, (value) -> inputs.deployPositionRotations = value); //only updates the value if the output is valid
        ifOk(deployMotor, deployEncoder::getVelocity, (value) -> inputs.deployVelocityRotationsPerSec = value);
        ifOk(
            deployMotor,
            new DoubleSupplier[] {deployMotor::getAppliedOutput, deployMotor::getBusVoltage},
            (values) -> inputs.deployAppliedVolts = values[0] * values[1]);
        ifOk(deployMotor, deployMotor::getOutputCurrent, (value) -> inputs.deployCurrentAmps = value);

        double desiredVoltage = this.controller.calculate(inputs.deployPositionRotations);

        Logger.recordOutput("OverBumperIntake/PIDRequestedVoltage", desiredVoltage);

        if (this.isClosed){
            this.setDeployVoltage(desiredVoltage);
        }

        inputs.isClosed = this.isClosed;
    }

    @Override
    public void resetEncoder(double positionRad) {
        deployEncoder.setPosition(positionRad);
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
    public void updateSetpoint(double setpoint) {
        this.motorSetpoint = this.clipSetpoint(setpoint);
        this.controller.setSetpoint(motorSetpoint);
    }

    public double clipSetpoint(double setpoint) {
        if (setpoint > OverBumperIntakeConstants.kRetractedRotations) {
            return OverBumperIntakeConstants.kRetractedRotations;
        } else if (setpoint < OverBumperIntakeConstants.kDeployedRotations) {
            return OverBumperIntakeConstants.kDeployedRotations;
        }
        return setpoint;
    }

    @Override
    public void setClosedLoop(boolean val) {
        this.isClosed = val;
    }

    @Override
    public void intake() {
        setIntakeVoltage(kIntakeVoltage * 12.0);
    }

    @Override
    public void outtake() {
        setIntakeVoltage(-kIntakeVoltage * 12.0);
    }

    @Override
    public void setPID(double kp, double ki, double kd) {
        this.controller.setPID(kp, ki, kd);
    }
}