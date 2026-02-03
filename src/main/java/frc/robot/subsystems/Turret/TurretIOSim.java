package frc.robot.subsystems.Turret;

import static frc.robot.subsystems.Turret.TurretConstants.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.math.controller.PIDController;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.math.MathUtil;

public class TurretIOSim implements TurretIO {

    private DCMotorSim motor = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), 0.004, 1), 
            DCMotor.getNEO(1));


    private double appliedVolts = 0.0;
    private static DIOSim turretLimitSwitchPressed  = new DIOSim(0); // Assuming digital input 0 is used for limit switch

    @AutoLogOutput(key="Turret/Angle")
    private double turretRadians = Units.degreesToRadians(0.0);

    private LoggedNetworkNumber tuningP = new LoggedNetworkNumber("Tuning/Turret/P", kProportionalTermSim);
    private LoggedNetworkNumber tuningD = new LoggedNetworkNumber("Tuning/Turret/I", kDerivativeTermSim);
    
    private PIDController turretController = new PIDController(tuningP.get(), 0.0, tuningD.get());
            @Override
            public void updateInputs(TurretIOInputs inputs) {
                
            if (tuningP.get() != turretController.getP() || tuningD.get() != turretController.getD()) {
                turretController.setPID(tuningP.get(), 0.0, tuningD.get());
    
                double absoluteRadians = this.motor.getAngularPositionRad() * kTurretGearRatio;
    
                motor.setInputVoltage(turretController.calculate(absoluteRadians));
    
               turretLimitSwitchPressed.setValue(true);
               motor.setInputVoltage(appliedVolts);
               motor.update(0.02); // Update simulation with a timestep of 20ms
    
               inputs.turretPositionRad = motor.getAngularPositionRad() * kTurretGearRatio;
               inputs.turretVelocityRadPerSec = motor.getAngularVelocityRadPerSec() * kTurretGearRatio;
               inputs.turretAppliedVoltage = appliedVolts;
               inputs.turretCurrentAmps = motor.getCurrentDrawAmps();
    
               inputs.turretLimitSwitchPressed = false;
            }
    
            }
    
            public boolean getSensor(DIOSim sensor){
                return sensor.getValue();
            }
    
            public void setTurretVoltage(double volts) {
                appliedVolts = MathUtil.clamp(volts, -12, 12);
            }
    
            @Override
            public void updateTurretPosition(double positionRad){
                motor.setAngle(positionRad);
            }
        }

       

       
    

