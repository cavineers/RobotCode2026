package frc.robot.subsystems.InBumperIntake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class InBumperIntake extends SubsystemBase {
    public final InBumperIntakeIO io;
    public final InBumperIntakeIOInputsAutoLogged inputs = new InBumperIntakeIOInputsAutoLogged();

    public InBumperIntake(InBumperIntakeIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("InBumperIntake", inputs);
    }

    public void setBottomVoltage(double volts) {
        io.setBottomVoltage(volts);
    }

    public void setHopperVoltage(double volts) {
        io.setHopperVoltage(volts);
    }

    public void setTopVoltage(double volts) {
        io.setTopVoltage(volts);
    }

    public Command setBottomVoltageCommand(double volts) {
        return Commands.run(() -> io.setBottomVoltage(volts), this).finallyDo(interrupted -> io.setBottomVoltage(0));
    }

    public Command setHopperVoltageCommand(double volts) {
        return Commands.run(() -> io.setHopperVoltage(volts), this).finallyDo(interrupted -> io.setHopperVoltage(0));
    }

    public Command setTopVoltageCommand(double volts) {
        return Commands.run(() -> io.setTopVoltage(volts), this).finallyDo(interrupted -> io.setTopVoltage(0.0));
    }

}
