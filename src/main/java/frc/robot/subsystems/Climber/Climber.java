package frc.robot.subsystems.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import static frc.robot.subsystems.Climber.ClimberConstants.*;
import frc.robot.subsystems.Climber.ClimberIOKraken;

import org.littletonrobotics.junction.AutoLogOutput;
import frc.robot.Constants;
import frc.robot.subsystems.Climber.ClimberConstants;

public class Climber extends SubsystemBase {
    private final ClimberIO io;
    private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

    private double kP;
    private double kI;
    private double kD;

    public enum ClimbState{
        RESTING,
        DEPLOYED,
        ENGAGED
    }
    
    @AutoLogOutput(key="Climber/ClimbState")
    public ClimbState climbState = ClimbState.RESTING;

    private LoggedNetworkNumber tuningP = new LoggedNetworkNumber("/Tuning/Climber/kP", ClimberConstants.kP);
    private LoggedNetworkNumber tuningI = new LoggedNetworkNumber("/Tuning/Climber/kI", ClimberConstants.kI);
    private LoggedNetworkNumber tuningD = new LoggedNetworkNumber("/Tuning/Climber/kD", ClimberConstants.kD);
    
    public Climber(ClimberIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        if (kP != tuningP.get() || kI != tuningI.get() || kD != tuningD.get()) {
            kP = tuningP.get();
            kI = tuningI.get();
            kD = tuningD.get();
            this.io.setPID(kP, kI, kD);
        }
        Logger.processInputs("Climber", inputs);
    }

    public Command setVoltageCommand(double volts) {
        if (Constants.currentMode != Constants.simMode){
            return Commands.runOnce(() -> {
                io.setClimberVoltage(volts);
            }, this);
        }
        return Commands.runOnce(() -> {
            io.setClimberVoltage(volts);
        }, this);
    } 
    public Command goToPresetCommand(double rotations) {
         return Commands.runOnce(() -> {
            io.updateClimberSetpoint(rotations);
        }, this);
    }

    public double getClimberPosition() {
        return inputs.climberPositionRotations;
    }

    public Command deployCommand() {
        return Commands.run(() -> {
            climbState = ClimbState.DEPLOYED;
            io.deploy();
            }, this).finallyDo(interrupted ->
            io.setClimberVoltage(0));
    }

    public Command retractCommand() {
        return Commands.run(() -> {
            climbState = ClimbState.RESTING;
            io.retract();
            }, this).finallyDo(interrupted ->
            io.setClimberVoltage(0));
    }

    public Command engageCommand() {
        return Commands.run(() -> {
            climbState = ClimbState.ENGAGED;
            io.engage();
            }, this).finallyDo(interrupted ->
            io.setClimberVoltage(0));
    }

}