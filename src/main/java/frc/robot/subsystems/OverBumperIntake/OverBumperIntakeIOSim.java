package frc.robot.subsystems.OverBumperIntake;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class OverBumperIntakeIOSim implements OverBumperIntakeIO {

    private DCMotorSim deployMotor = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), 0.004,1),
        DCMotor.getNEO(1));

    private DCMotorSim intakeMotor = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), 0.004, 1),
        DCMotor.getNEO(1));
    
    @AutoLogOutput(key="OverBumperIntake/motorSetpoint")
    private double motorSetpoint = 0;

    @AutoLogOutput(key="OverBumperIntake/Deployed")
    public boolean deployed = false;

    @AutoLogOutput(key="OverBumperIntake/IsClosed")
    private boolean isClosed = false;

    private double appliedVolts = 0.0;
    private PIDController simPID = new PIDController (OverBumperIntakeConstants.kSimP, OverBumperIntakeConstants.kSimI, OverBumperIntakeConstants.kSimD);
   
    @Override
    public void updateInputs(OverBumperIntakeIOInputs inputs) {
        deployMotor.setInputVoltage(appliedVolts);
        deployMotor.update(0.02);

        inputs.deployPositionRad = deployMotor.getAngularPositionRad();
        inputs.deployVelocityRadPerSec = deployMotor.getAngularVelocityRadPerSec();
        inputs.deployAppliedVolts = appliedVolts;
        inputs.deployCurrentAmps = deployMotor.getCurrentDrawAmps();

        intakeMotor.setInputVoltage(appliedVolts);
        intakeMotor.update(0.02);

        inputs.intakePositionRad = intakeMotor.getAngularPositionRad();
        inputs.intakeVelocityRadPerSec = intakeMotor.getAngularVelocityRadPerSec();
        inputs.intakeAppliedVolts = appliedVolts;
        inputs.intakeCurrentAmps = intakeMotor.getCurrentDrawAmps();

        for (int i = 0; i < inputs.recentAmpsHistory.length - 1; i++) {
            inputs.recentAmpsHistory[i] = inputs.recentAmpsHistory[i + 1];
        }
        // Set the last element to currentAmps
        inputs.recentAmpsHistory[inputs.recentAmpsHistory.length - 1] = inputs.deployCurrentAmps;

        double desiredVoltage = this.simPID.calculate(inputs.deployPositionRad);

        Logger.recordOutput("OverBumperIntake/PIDRequestedVoltage", desiredVoltage);

        if (this.isClosed){
            this.setDeployVoltage(desiredVoltage);
        }
    }

    @Override
    public void setDeployVoltage(double volts) {
        appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    }

    @Override
    public void setIntakeVoltage(double volts) {
        appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    }

    @Override
    public void updateSetpoint(double setpoint) {
        this.motorSetpoint = this.clipSetpoint(setpoint);
        this.simPID.setSetpoint(motorSetpoint);
    }

    public double clipSetpoint(double setpoint) {
        if (setpoint > OverBumperIntakeConstants.kDeployedRotations) {
            return OverBumperIntakeConstants.kDeployedRotations;
        } else if (setpoint < OverBumperIntakeConstants.kRetractedRotations) {
            return OverBumperIntakeConstants.kRetractedRotations;
        }
        return setpoint;
    }
    
    @Override
    public void setClosedLoop(boolean val) {
        this.isClosed = val;
    }

    @Override
    public void autoDeploy() {
        if (deployed) {
            updateSetpoint(OverBumperIntakeConstants.kRetractedRotations);
            this.deployed = false;
        } else {
            updateSetpoint(OverBumperIntakeConstants.kDeployedRotations);
            this.deployed = true;
        }
    }

    @Override
    public void intake() {
        setIntakeVoltage(OverBumperIntakeConstants.kIntakeVoltage * 12.0);
    }

    @Override
    public void outtake() {
        setIntakeVoltage(OverBumperIntakeConstants.kIntakeVoltage * -12.0);
    }

    @Override 
    public void setPID(double kp, double ki, double kd) {
        simPID.setP(kp);
        simPID.setI(ki);
        simPID.setD(kd);
    }
}

