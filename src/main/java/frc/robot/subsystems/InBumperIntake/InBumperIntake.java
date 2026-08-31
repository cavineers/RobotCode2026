package frc.robot.subsystems.InBumperIntake;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.subsystems.InBumperIntake.InBumperIntakeConstants.*;


public class InBumperIntake extends SubsystemBase {
    public enum IntakeState {
        IDLE,
        GROUND_TO_SHOOTER,
        GROUND_TO_HOPPER,
        HOPPER_TO_SHOOTER,
        MANUAL_CONTROL
    }

    public final InBumperIntakeIO io;
    public final InBumperIntakeIOInputsAutoLogged inputs = new InBumperIntakeIOInputsAutoLogged();
    
    @AutoLogOutput(key = "InBumperIntake/State")
    private IntakeState currentState = IntakeState.IDLE;

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

    public void setHopperVoltage(double volts) {
        io.setHopperVoltage(volts);
    }

    public void setSpindexerVoltage(double volts) {
        io.setHopperVoltage(volts);
    }
    
    public IntakeState getState() {
        return currentState;
    }

    public Command runGroundToShooter() {
        return this.run(() -> {
            currentState = IntakeState.GROUND_TO_SHOOTER;
            io.setOutsideVoltage(-kOutsideVoltage);
            io.setBottomVoltage(kBottomVoltage);
            io.setTopVoltage(kTopVoltage);
            io.setHopperVoltage(kHopperVoltage);
        }).finallyDo(interrupted -> {
            currentState = IntakeState.IDLE;
            io.setOutsideVoltage(0);
            io.setBottomVoltage(0);
            io.setTopVoltage(0);
            io.setHopperVoltage(0);
        });
    }

    public Command runGroundToHopper() {
        return this.run(() -> {
            currentState = IntakeState.GROUND_TO_HOPPER;
            io.setOutsideVoltage(-kOutsideVoltage);
            io.setBottomVoltage(kBottomVoltage);
            io.setTopVoltage(-kTopVoltage);
            io.setHopperVoltage(-kHopperVoltage);
        }).finallyDo(interrupted -> {
            currentState = IntakeState.IDLE;
            io.setOutsideVoltage(0);
            io.setBottomVoltage(0);
            io.setTopVoltage(0);
            io.setHopperVoltage(0);
        });
    }

    public Command runHopperToShooter() {
        return this.run(() -> {
            currentState = IntakeState.HOPPER_TO_SHOOTER;
            io.setOutsideVoltage(-kOutsideVoltage);
            io.setBottomVoltage(-kBottomVoltage);
            io.setTopVoltage(kTopVoltage);
            io.setHopperVoltage(kHopperVoltage); // Hopper runs during hopper to shooter
        }).finallyDo(interrupted -> {
            currentState = IntakeState.IDLE;
            io.setOutsideVoltage(0);
            io.setBottomVoltage(0);
            io.setTopVoltage(0);
            io.setHopperVoltage(0);
        });
    }

    public Command runIntakeAgitateSequence() {
    return Commands.sequence(
        runHopperToShooter().withTimeout(2.0),
        Commands.waitSeconds(0.25),
        runGroundToHopper().withTimeout(0.2),
        Commands.waitSeconds(0.25)
    ).repeatedly();
    }

    public Command stopCommand() {
        return this.runOnce(() -> {
            currentState = IntakeState.IDLE;
            io.setOutsideVoltage(0);
            io.setBottomVoltage(0);
            io.setTopVoltage(0);
            io.setHopperVoltage(0);
        });
    }

    public void stopIntake() {
        currentState = IntakeState.IDLE;
        io.setOutsideVoltage(0);
        io.setBottomVoltage(0);
        io.setTopVoltage(0);
        io.setHopperVoltage(0);
    }

    public void setState(IntakeState state) {
        currentState = state;
    }

}
