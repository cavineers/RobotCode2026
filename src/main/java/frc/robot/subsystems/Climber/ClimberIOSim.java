package frc.robot.subsystems.Climber;

import static frc.robot.subsystems.Climber.ClimberConstants.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.math.controller.PIDController;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.math.MathUtil;

public class ClimberIOSim implements ClimberIO {

    private DCMotorSim motor = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), 0.004, 1), 
            DCMotor.getNEO(1));


    private double appliedVolts = 0.0;
    @AutoLogOutput(key="Climber/Setpoint")
    private double climberSetpoint = 0.0;

    private LoggedNetworkNumber tuningP = new LoggedNetworkNumber("Tuning/Dealgaefier/P", kProportionalTermSim);
    private LoggedNetworkNumber tuningD = new LoggedNetworkNumber("Tuning/Dealgaefier/I", kDerivativeTermSim);

    private static DIOSim limitSwitch = new DIOSim(kLimitSwitchID);

    private PIDController climberController = new PIDController(tuningP.get(), 0.0, tuningD.get());

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
            
        if (tuningP.get() != climberController.getP() || tuningD.get() != climberController.getD()) {
            climberController.setPID(tuningP.get(), 0.0, tuningD.get());

            double absoluteRotations = this.motor.getAngularPositionRotations() * kClimberGearRatio;

            motor.setInputVoltage(climberController.calculate(absoluteRotations));

            motor.update(0.2);

           motor.setInputVoltage(appliedVolts);
           motor.update(0.02); // Update simulation with a timestep of 20ms

           inputs.climberPositionRotations = motor.getAngularPositionRad();
           inputs.climberVelocityRotationsPerSec = motor.getAngularVelocityRadPerSec();
           inputs.climberAppliedVoltage = appliedVolts;
           inputs.climberCurrentAmps = motor.getCurrentDrawAmps();

        }

    }

    public void setClimberVolts(double volts) {
        appliedVolts = MathUtil.clamp(volts, -12, 12);
    }

    public boolean getLimitSwitch() {
        return limitSwitch.getValue();
    }
    
    @Override
    public void updateClimberSetpoint(double positionRad){
        this.climberSetpoint = positionRad;
        climberController.setSetpoint(this.climberSetpoint);
    }
            

}

       

       
    
