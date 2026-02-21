package frc.robot.subsystems.OverBumperIntake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;
import static frc.robot.subsystems.OverBumperIntake.OverBumperIntakeConstants.*;


public class OverBumperIntake extends SubsystemBase {
    private final OverBumperIntakeIO io;
    private final OverBumperIntakeIOInputsAutoLogged inputs = new OverBumperIntakeIOInputsAutoLogged();

    private double kP;
    private double kI;
    private double kD;
    
    private LoggedNetworkNumber tuningP = new LoggedNetworkNumber("/Tuning/OverBumperIntake/kP", OverBumperIntakeConstants.kProportionalGainSpark);  
    private LoggedNetworkNumber tuningI = new LoggedNetworkNumber ("/Tuning/OverBumperIntake/kI", OverBumperIntakeConstants.kIntegralTermSpark);  
    private LoggedNetworkNumber tuningD = new LoggedNetworkNumber ("/Tuning/OverBumperIntake/kD", OverBumperIntakeConstants.kDerivativeTermSpark); 

    public OverBumperIntake(OverBumperIntakeIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);

        if (kP != tuningP.get() || kI != tuningI.get() || kD != tuningD.get()) {
            kP = tuningP.get();
            kI = tuningI.get();
            kD = tuningD.get();
            io.setPID(kP, kI, kD);
        }
        Logger.processInputs("OverBumperIntake", inputs);

        // if (inputs.cutoff && inputs.deployed && inputs.isClosed) {
        //     io.resetEncoder(OverBumperIntakeConstants.kHomingSwitchZeroPositionRot);
        // }
    }

    public void setIntakeVoltage(double volts) {
        io.setIntakeVoltage(volts);
    }

    public void setDeployVoltage(double volts) {
        io.setDeployVoltage(volts);
    }

    public Command setDeployVoltageCommand(double volts) {
        return Commands.run(() -> {
            io.setClosedLoop(false);
            io.setDeployVoltage(volts);
        }, this);
    }

    public Command setIntakeVoltageCommand(double volts) {
        return Commands.run(() -> io.setIntakeVoltage(volts), this).finallyDo(interrupted -> io.setIntakeVoltage(0.0));
    }

    public Command deployCommand() {
        return Commands.runOnce(() -> {
            this.io.setClosedLoop(true);
            io.autoDeploy();
        }, this);
    }

    public Command intakeCommand() {
        return Commands.run(() -> io.intake(), this).finallyDo(interrupted -> io.setIntakeVoltage(0.0));
    }

    public Command outtakeCommand() {
        return Commands.run(() -> io.outtake(), this).finallyDo(interrupted -> io.setIntakeVoltage(0.0));
    }
}
