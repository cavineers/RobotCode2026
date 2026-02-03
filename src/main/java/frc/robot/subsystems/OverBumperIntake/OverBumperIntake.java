package frc.robot.subsystems.OverBumperIntake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import static frc.robot.subsystems.OverBumperIntake.OverBumperIntakeConstants.*;


public class OverBumperIntake extends SubsystemBase {
    private final OverBumperIntakeIO io;
    private final OverBumperIntakeIOInputsAutoLogged inputs = new OverBumperIntakeIOInputsAutoLogged();

    public OverBumperIntake(OverBumperIntakeIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);

        double sum = 0;
        for (double value : inputs.recentAmpsHistory) {
            sum += value;
        }
        Logger.recordOutput("OverBumperIntake/AverageAmps", sum / inputs.recentAmpsHistory.length);
        if (sum / inputs.recentAmpsHistory.length > kCutOffAmps) {
            io.setDeployVoltage(0.0);
            Logger.recordOutput("OverBumperIntake/CutOff", true);
        } else {
            Logger.recordOutput("OverBumperIntake/CutOff", false);
        }
        
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

    public Command intakeCommand() {
        return Commands.run(() -> io.intake(), this).finallyDo(interrupted -> io.setIntakeVoltage(0.0));
    }

    public Command outtakeCommand() {
        return Commands.run(() -> io.outtake(), this).finallyDo(interrupted -> io.setIntakeVoltage(0.0));
    }
}
