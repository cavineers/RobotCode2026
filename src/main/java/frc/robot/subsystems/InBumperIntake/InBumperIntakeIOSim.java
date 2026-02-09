package frc.robot.subsystems.InBumperIntake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class InBumperIntakeIOSim implements InBumperIntakeIO {
    
    private DCMotorSim bottomMotor = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), 0.004,1),
        DCMotor.getNEO(1));

    private DCMotorSim hopperMotor = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), 0.004,1),
        DCMotor.getNEO(1));
    
    private DCMotorSim topMotor = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), 0.004,1),
        DCMotor.getNEO(1));

    private double bottomMotorAppliedVolts = 0.0;
    private double hopperMotorAppliedVolts = 0.0;
    private double topMotorAppliedVolts = 0.0;

    @Override
    public void updateInputs(InBumperIntakeIOInputs inputs) {
        bottomMotor.setInputVoltage(bottomMotorAppliedVolts);
        hopperMotor.setInputVoltage(hopperMotorAppliedVolts);
        topMotor.setInputVoltage(topMotorAppliedVolts);

        bottomMotor.update(0.02);
        hopperMotor.update(0.02);
        topMotor.update(0.02);

        inputs.bottomMotorPositionRad = bottomMotor.getAngularPositionRad();
        inputs.bottomMotorVelocityRadPerSec = bottomMotor.getAngularVelocityRadPerSec();
        inputs.bottomMotorAppliedVolts = bottomMotorAppliedVolts;
        inputs.bottomMotorCurrentAmps = bottomMotor.getCurrentDrawAmps();

        inputs.hopperMotorPositionRad = hopperMotor.getAngularPositionRad();
        inputs.hopperMotorVelocityRadPerSec = hopperMotor.getAngularVelocityRadPerSec();
        inputs.hopperMotorAppliedVolts = hopperMotorAppliedVolts;
        inputs.hopperMotorCurrentAmps = hopperMotor.getCurrentDrawAmps();

        inputs.topMotorPositionRad = topMotor.getAngularPositionRad();
        inputs.topMotorVelocityRadPerSec = topMotor.getAngularVelocityRadPerSec();
        inputs.topMotorAppliedVolts = topMotorAppliedVolts;
        inputs.topMotorCurrentAmps = topMotor.getCurrentDrawAmps();
    }

    @Override
    public void setBottomVoltage(double volts) {
        bottomMotorAppliedVolts = MathUtil.clamp(volts, -12, 12);
    }

    @Override
    public void setHopperVoltage(double volts) {
        hopperMotorAppliedVolts = MathUtil.clamp(volts, -12, 12);
    }

    @Override
    public void setTopVoltage(double volts) {
        topMotorAppliedVolts = MathUtil.clamp(volts, -12, 12);
    }

}