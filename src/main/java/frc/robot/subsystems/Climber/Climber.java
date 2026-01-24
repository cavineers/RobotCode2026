package frc.robot.subsystems.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {
    private final ClimberIO io;
    private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

    public Climber(ClimberIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Climber", inputs);
    }

    public Command setReachVoltageCommand(double volts) {
        return Commands.run(() -> io.setReachVoltage(volts), this).finallyDo(interrupted -> io.setReachVoltage(0));
    }

    public Command deployCommand() {
        return Commands.run(() -> io.deploy(), this).finallyDo(interrupted -> io.setIntakeVoltage(0));
    }

    public Command retractCommand() {
        return Commands.run(() -> io.retract(), this).finallyDo(interrupted -> io.setIntakeVoltage(0));
    }

    public void setDeployVoltage(double volts) {
        io.setDeployVoltage(volts);
    }

    public void setRetractVoltage(double volts) {
        io.setRetractVoltage(volts);
    }
}