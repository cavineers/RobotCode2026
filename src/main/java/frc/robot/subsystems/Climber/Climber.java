package frc.robot.subsystems.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

import static frc.robot.subsystems.Climber.ClimberConstants.kGravityTermSpark;

import org.littletonrobotics.junction.AutoLogOutput;
import frc.robot.Constants;
import frc.robot.subsystems.Climber.ClimberConstants;

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

    public Command setVoltageCommand(double volts) {
        this.io.setClosedLoop(false);
        if (Constants.currentMode != Constants.simMode){
            return Commands.run(() -> {
                io.setClosedLoop(false);
                io.setDeployVoltage(volts + kGravityTermSpark); //kG throwing off?????
            }, this);
        }
        return Commands.run(() -> {
            io.setClosedLoop(false);
            io.setDeployVoltage(volts);
        }, this);
    } 
    public Command goToPresetCommand(double rotations) {
         return Commands.run(() -> {
            this.io.setClosedLoop(true);
            io.updateClimberSetpoint(rotations);
        }, this);
    }

    public double getClimberPosition() {
        return inputs.climberPositionRotations;
    }

    public Command deployCommand() {
        return Commands.run(() -> io.deploy(), this).finallyDo(interrupted -> io.setDeployVoltage(0));
    }

    public Command retractCommand() {
        return Commands.run(() -> io.retract(), this).finallyDo(interrupted -> io.setRetractVoltage(0));
    }

    public void setDeployVoltage(double volts) {
        io.setDeployVoltage(volts);
    }

    public void setRetractVoltage(double volts) {
        io.setRetractVoltage(volts);
    }
}