package frc.robot.subsystems.Climber;

import static frc.lib.SparkUtil.*;

import static frc.robot.subsystems.Climber.ClimberConstants.*;
import static frc.robot.subsystems.Climber.Climber.*;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class ClimberIOSpark implements ClimberIO {
    final SparkMax deployMotor = new SparkMax(kClimberCanID, MotorType.kBrushless);
    private final RelativeEncoder deployEncoder = deployMotor.getEncoder();

    private LoggedNetworkNumber tuningP = new LoggedNetworkNumber("/Tuning/Climber/P", ClimberConstants.kProportionalGainSpark);
    private LoggedNetworkNumber tuningD = new LoggedNetworkNumber("/Tuning/Climber/D", ClimberConstants.kDerivativeTermSpark);
    private LoggedNetworkNumber tuningG = new LoggedNetworkNumber("/Tuning/Climber/G", ClimberConstants.kGravityTermSpark); 

    @AutoLogOutput(key="Climber/Setpoint")
    private double absSetpoint;

    @AutoLogOutput(key="Climber/IsClosed")
    private boolean isClosed = true;

    private PIDController controller = new PIDController(kProportionalGainSpark, kIntegralTermSpark, kDerivativeTermSpark);

    private SparkMaxConfig deployConfig;

        @AutoLogOutput(key="Climber/Deployed")
    public boolean deployed = false;
    
    public ClimberIOSpark(){
        this.controller.enableContinuousInput(0, 1);
    
        deployConfig = new SparkMaxConfig();
        deployConfig
            
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(ClimberConstants.kCurrentLimit)    
            .voltageCompensation(12); 
     
        tryUntilOk(
            deployMotor,
            5,
            () -> deployMotor.configure(deployConfig, ResetMode.kResetSafeParameters,
                    PersistMode.kPersistParameters));

        this.controller.setTolerance(kTolerance); // doesn't actually do anything unless you are using controller.atSetpoint()

    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {

        ifOk(deployMotor, deployEncoder::getPosition, (value) -> inputs.climberPositionRotations = value);
        ifOk(deployMotor, deployEncoder::getVelocity, (value) -> inputs.climberVelocityRotationsPerSec = value);
        ifOk(
            deployMotor,
            new DoubleSupplier[] {deployMotor::getAppliedOutput, deployMotor::getBusVoltage},
            (values -> inputs.climberAppliedVoltage = values[0] * values[1]));
        ifOk(deployMotor, deployMotor::getOutputCurrent, (value) -> inputs.climberCurrentAmps = value);

        double desiredVoltage = this.controller.calculate(inputs.climberPositionRotations) + this.calculateFeedforward();
        if (this.isClosed){
            this.setDeployVoltage(desiredVoltage);
        }

        if (ClimberConstants.kTuningMode){
            this.updatePID();
        }
        inputs.deployed = this.deployed;
    }
    
    @Override
    public void setDeployVoltage(double volts) {
        deployMotor.setVoltage(volts);
    }

    public void updateClimberSetpoint(double setpoint) {
        this.absSetpoint = this.clipSetpoint(setpoint);
        this.controller.setSetpoint(absSetpoint);
    }

    public double clipSetpoint(double setpoint) {
        // if(absSetpoint < ClimberConstants.kDeployedAbsoluteRotations) {
        //     return ClimberConstants.kDeployedAbsoluteRotations;
        // } else if(absSetpoint < ClimberConstants.kRestAbsoluteRotations) {
        //     return ClimberConstants.kRestAbsoluteRotations;
        // }
        return setpoint;
    }

    @Override
    public void setClosedLoop(boolean val) {
        this.isClosed = val;
    }


    private double calculateFeedforward() {
        double feedforward = ClimberConstants.kTuningMode ? this.tuningG.get() : ClimberConstants.kGravityTermSpark;
        return feedforward;
    }

    private void updatePID() {
        double currentP = this.controller.getP();
        double currentD = this.controller.getD();

        if (currentP != this.tuningP.get() || currentD != this.tuningD.get()){
            this.controller.setPID(this.tuningP.get(), 0, this.tuningD.get());
        }
    }

    public void deploy() {
        updateClimberSetpoint(kDeployedAbsoluteRotations);
        deployed = true;
    }

    public void retract() {
        updateClimberSetpoint(kRestAbsoluteRotations);
        deployed = false;
    }
 }
