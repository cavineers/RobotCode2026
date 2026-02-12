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

    public void setTopVoltage(double volts) {
        io.setTopVoltage(volts);
    }

    public void setOutsideVoltage(double volts) {
        io.setOutsideVoltage(volts);
    }

    public Command setOutsideVoltageCommand(double volts) {
        return Commands.run(() -> io.setOutsideVoltage(volts)).finallyDo(interrupted -> io.setOutsideVoltage(0.0));
    }

    public Command setBottomVoltageCommand(double volts) {
        return Commands.run(() -> io.setBottomVoltage(volts)).finallyDo(interrupted -> io.setBottomVoltage(0));
    }

    public Command setTopVoltageCommand(double volts) {
        return Commands.run(() -> io.setTopVoltage(volts)).finallyDo(interrupted -> io.setTopVoltage(0));
    }

    public Command runGroundToShooter(double outsideVolts, double bottomVolts, double topVolts) {
        return Commands.run(() -> {
            io.setOutsideVoltage(-outsideVolts);
            io.setBottomVoltage(bottomVolts);
            io.setTopVoltage(topVolts);})
        .finallyDo(interrupted -> {
            io.setOutsideVoltage(0);
            io.setBottomVoltage(0);
            io.setTopVoltage(0);
        });
    }

    public Command runGroundToHopper(double outsideVolts, double bottomVolts, double topVolts) {
        return Commands.run(() -> {
            io.setOutsideVoltage(-outsideVolts);
            io.setBottomVoltage(bottomVolts);
            io.setTopVoltage(-topVolts);})
        .finallyDo(interrupted -> {
            io.setOutsideVoltage(0);
            io.setBottomVoltage(0);
            io.setTopVoltage(0);
        });
    }

    public Command runHopperToShooter(double outsideVolts, double bottomVolts, double topVolts) {
        return Commands.run(() -> {
            io.setOutsideVoltage(-outsideVolts);
            io.setBottomVoltage(-bottomVolts);
            io.setTopVoltage(topVolts);})
        .finallyDo(interrupted -> {
            io.setOutsideVoltage(0);
            io.setBottomVoltage(0);
            io.setTopVoltage(0);
        });
    }

}
