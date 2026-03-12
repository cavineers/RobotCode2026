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
    
    @AutoLogOutput(key = "OverBumperIntake/deployed")
    private boolean deployed = false;

    
    private LoggedNetworkNumber tuningP = new LoggedNetworkNumber("/Tuning/OverBumperIntake/kP", OverBumperIntakeConstants.kProportionalGainSpark);  
    private LoggedNetworkNumber tuningI = new LoggedNetworkNumber ("/Tuning/OverBumperIntake/kI", OverBumperIntakeConstants.kIntegralTermSpark);  
    private LoggedNetworkNumber tuningD = new LoggedNetworkNumber ("/Tuning/OverBumperIntake/kD", OverBumperIntakeConstants.kDerivativeTermSpark); 

    public OverBumperIntake(OverBumperIntakeIO io) {
        this.io = io;
        io.updateSetpoint(kRetractedRotations);
        io.setClosedLoop(true);
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
        Logger.recordOutput("OverBumperIntake/deployed", deployed);
    }

    public void setIntakeVoltage(double volts) {
        io.setIntakeVoltage(volts);
    }

    public void setDeployVoltage(double volts) {
        io.setDeployVoltage(volts);
    }

    public Command deployCommand() {
        return Commands.runOnce(() -> {
            if (deployed) {
                // Retract
                io.updateSetpoint(kRetractedRotations);
                io.setIntakeVoltage(0);
                deployed = false;
            } else {
                // Deploy
                io.updateSetpoint(kDeployedRotations);
                io.intake();
                deployed = true;
            }
        }, this);
    }

    public Command intakeCommand() {
        return Commands.run(() -> io.intake(), this).finallyDo(interrupted -> io.setIntakeVoltage(0.0));
    }

    public Command outtakeCommand() {
        return Commands.run(() -> io.outtake(), this).finallyDo(interrupted -> io.setIntakeVoltage(0.0));
    }

    /**
     * Unjam sequence: repeatedly deploys and retracts the OTB arm while running
     * the OTB intake wheel, then retracts and stops on cancel.
     */
    public Command unjamCommand() {
        final double CYCLE_SECS = 1.0;

        Command cmd = Commands.sequence(
            // Start OTB intake wheel immediately
            Commands.runOnce(() -> io.intake()),
            // Continuously cycle deploy -> retract until cancelled
            Commands.repeatingSequence(
                Commands.runOnce(() -> { io.updateSetpoint(kDeployedRotations); deployed = true; }),
                Commands.waitSeconds(CYCLE_SECS),
                Commands.runOnce(() -> { io.updateSetpoint(kRetractedRotations); deployed = false; }),
                Commands.waitSeconds(CYCLE_SECS)
            )
        ).finallyDo(() -> {
            // Always retract and stop on end
            io.updateSetpoint(kRetractedRotations);
            io.setIntakeVoltage(0);
            deployed = false;
        }).withName("OTB Unjam");
        cmd.addRequirements(this);
        return cmd;
    }
}
