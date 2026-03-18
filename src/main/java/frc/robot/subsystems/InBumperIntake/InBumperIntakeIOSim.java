package frc.robot.subsystems.InBumperIntake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.InBumperIntake.InBumperIntakeIO;

public class InBumperIntakeIOSim implements InBumperIntakeIO {
    
    private DCMotorSim bottomMotor = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), 0.004,1),
        DCMotor.getNEO(1));

    private DCMotorSim topMotor = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), 0.004,1),
        DCMotor.getNEO(1));
    
    private DCMotorSim outsideMotor = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), 0.004,1),
        DCMotor.getNEO(1));

    private DCMotorSim indexerMotor = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getNeoVortex(1), 0.004,1),
        DCMotor.getNeoVortex(1));

    private double bottomMotorAppliedVolts = 0.0;
    private double topMotorAppliedVolts = 0.0;
    private double outsideMotorAppliedVolts = 0.0;
    private double indexerMotorAppliedVolts = 0.0;

    @Override
    public void updateInputs(InBumperIntakeIOInputs inputs) {
        bottomMotor.setInputVoltage(bottomMotorAppliedVolts);
        topMotor.setInputVoltage(topMotorAppliedVolts);
        outsideMotor.setInputVoltage(outsideMotorAppliedVolts);
        indexerMotor.setInputVoltage(indexerMotorAppliedVolts);

        bottomMotor.update(0.02);
        topMotor.update(0.02);
        outsideMotor.update(0.02);
        indexerMotor.update(0.02);

        inputs.bottomMotorPositionRotations = bottomMotor.getAngularPositionRotations();
        inputs.bottomMotorVelocityRPM = bottomMotor.getAngularVelocityRPM();
        inputs.bottomMotorAppliedVolts = bottomMotorAppliedVolts;
        inputs.bottomMotorCurrentAmps = bottomMotor.getCurrentDrawAmps();

        inputs.topMotorPositionRotations = topMotor.getAngularPositionRotations();
        inputs.topMotorVelocityRPM = topMotor.getAngularVelocityRPM();
        inputs.topMotorAppliedVolts = topMotorAppliedVolts;
        inputs.topMotorCurrentAmps = topMotor.getCurrentDrawAmps();

        inputs.outsideMotorPositionRotations = outsideMotor.getAngularPositionRotations();
        inputs.outsideMotorVelocityRPM = outsideMotor.getAngularVelocityRPM();
        inputs.outsideMotorAppliedVolts = outsideMotorAppliedVolts;
        inputs.outsideMotorCurrentAmps = outsideMotor.getCurrentDrawAmps();

        inputs.indexerMotorPositionRotations = indexerMotor.getAngularPositionRotations();
        inputs.indexerMotorVelocityRPM = indexerMotor.getAngularVelocityRPM();
        inputs.indexerMotorAppliedVolts = indexerMotorAppliedVolts;
        inputs.indexerMotorCurrentAmps = indexerMotor.getCurrentDrawAmps();
    }

    @Override
    public void setBottomVoltage(double volts) {
        bottomMotorAppliedVolts = MathUtil.clamp(volts, -12, 12);
    }

    @Override
    public void setTopVoltage(double volts) {
        topMotorAppliedVolts = MathUtil.clamp(volts, -12, 12);
    }

    @Override
    public void setOutsideVoltage(double volts) {
        outsideMotorAppliedVolts = MathUtil.clamp(volts, -12, 12);
    }

    @Override
    public void setIndexerVoltage(double volts) {
        indexerMotorAppliedVolts = MathUtil.clamp(volts, -12, 12);
    }

}