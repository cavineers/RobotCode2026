package frc.robot.subsystems.Climber;

import static frc.robot.subsystems.Climber.ClimberConstants.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.math.controller.PIDController;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.math.MathUtil;

public class ClimberIOSim implements ClimberIO {
    @AutoLogOutput(key="Climber/Setpoint")
    private double absSetpoint = 0;

    private boolean tempCutoff = false;
    
    public enum ClimbState{
        RESTING,
        DEPLOYED,
        ENGAGED
    }
    
    @AutoLogOutput(key="Climber/ClimbState")
    private ClimbState climbState = ClimbState.RESTING;

    private DCMotorSim climberMotor = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), 0.004, 1), 
            DCMotor.getNEO(1));

    @AutoLogOutput(key="Climber/Setpoint")
    private double climberSetpoint = 0.0;

    private LoggedNetworkNumber tuningP = new LoggedNetworkNumber("Tuning/Dealgaefier/P", kProportionalTermSim);
    private LoggedNetworkNumber tuningD = new LoggedNetworkNumber("Tuning/Dealgaefier/I", kDerivativeTermSim);

    private double climberAppliedVoltage = 0.0;
    private PIDController simPID = new PIDController(tuningP.get(), 0.0, tuningD.get());

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        climberMotor.setInputVoltage(climberAppliedVoltage);
        climberMotor.update(0.02); // Update simulation with a timestep of 20ms

        inputs.climberVelocityRotationsPerSec = climberMotor.getAngularVelocityRadPerSec()/(2*Math.PI);;
        inputs.climberAppliedVoltage = climberAppliedVoltage;
        inputs.climberCurrentAmps = climberMotor.getCurrentDrawAmps();
        inputs.climberPositionRotations = climberMotor.getAngularPositionRotations();
        inputs.cutoff = this.tempCutoff;
        
        Logger.recordOutput("Climber/climberPositionRotations", inputs.climberPositionRotations);

        for (int i = 0; i < inputs.recentAmpsHistory.length - 1; i++) {
            inputs.recentAmpsHistory[i] = inputs.recentAmpsHistory[i + 1];
        }
        // Set the last element to currentAmps
        inputs.recentAmpsHistory[inputs.recentAmpsHistory.length - 1] = inputs.climberCurrentAmps;

        double sum = 0;
        for (double value : inputs.recentAmpsHistory) {
            sum += value;
        }
        Logger.recordOutput("OverBumperIntake/AverageAmps", sum / inputs.recentAmpsHistory.length);
        if (sum / inputs.recentAmpsHistory.length > kCutOffAmps) {
            tempCutoff = true;
        } else {
            tempCutoff = false;
        }
    }

    @Override
    public void resetEncoder(double rotations) {
        climberMotor.setAngle(rotations);
    }

    @Override
    public void updateClimberSetpoint(double setpoint) {
        this.absSetpoint = this.clipSetpoint(setpoint);
        this.simPID.setSetpoint(absSetpoint);
        climberAppliedVoltage =
        MathUtil.clamp(simPID.calculate(climberMotor.getAngularPositionRotations()), -12.0, 12.0);
    }

    public double clipSetpoint(double setpoint) {
        if (setpoint > ClimberConstants.kDeployedMotorRotations) {
            return ClimberConstants.kDeployedMotorRotations;
        }
        else if (setpoint < ClimberConstants.kRestMotorRotations) {
             return ClimberConstants.kRestMotorRotations;
        }
        return setpoint;
    }

    @Override
    public void setClimberVoltage(double volts) {
        climberAppliedVoltage = MathUtil.clamp(volts, -12.0, 12.0);
    }

    @Override
    public void setPID(double kP, double kI, double kD) {
        simPID.setP(kP);
        simPID.setI(kI);
        simPID.setD(kD);
    }
}