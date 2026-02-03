package frc.robot.subsystems.OverBumperIntake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class OverBumperIntake extends SubsystemBase {
    private final OverBumperIntakeIO io;
    private final OverBumperIntakeIOInputsAutoLogged inputs = new OverBumperIntakeIOInputsAutoLogged();

    public OverBumperIntake(OverBumperIntakeIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("OverBumperIntake", inputs);
    }

    public void setIntakeVoltage(double volts) {
        io.setIntakeVoltage(volts);
    }

    public void setDeployVoltage(double volts) {
        io.setDeployVoltage(volts);
    }

    public Command deployCommand() {
        return Commands.run(() -> io.deploy(), this).finallyDo(interrupted -> io.setDeployVoltage(0));
    }

    public Command retractCommand() {
        return Commands.run(() -> io.retract(), this).finallyDo(interrupted -> io.setDeployVoltage(0));
    }

    public Command intakeCommand(double volts) {
        return Commands.run(() -> io.intake(), this).finallyDo(interrupted -> io.setIntakeVoltage(0.0));
    }
}
