package frc.robot.subsystems.InBumperIntake;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


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
        currentState = IntakeState.MANUAL_CONTROL;
    }

    public void setTopVoltage(double volts) {
        io.setTopVoltage(volts);
        currentState = IntakeState.MANUAL_CONTROL;
    }

    public void setOutsideVoltage(double volts) {
        io.setOutsideVoltage(volts);
        currentState = IntakeState.MANUAL_CONTROL;
    }
    
    public IntakeState getState() {
        return currentState;
    }

    public Command runGroundToShooter(double outsideVolts, double bottomVolts, double topVolts) {
        return this.run(() -> {
            currentState = IntakeState.GROUND_TO_SHOOTER;
            io.setOutsideVoltage(-outsideVolts);
            io.setBottomVoltage(bottomVolts);
            io.setTopVoltage(topVolts);
        }).finallyDo(interrupted -> {
            currentState = IntakeState.IDLE;
            io.setOutsideVoltage(0);
            io.setBottomVoltage(0);
            io.setTopVoltage(0);
        });
    }

    public Command runGroundToHopper(double outsideVolts, double bottomVolts, double topVolts) {
        return this.run(() -> {
            currentState = IntakeState.GROUND_TO_HOPPER;
            io.setOutsideVoltage(-outsideVolts);
            io.setBottomVoltage(bottomVolts);
            io.setTopVoltage(-topVolts);
        }).finallyDo(interrupted -> {
            currentState = IntakeState.IDLE;
            io.setOutsideVoltage(0);
            io.setBottomVoltage(0);
            io.setTopVoltage(0);
        });
    }

    public Command runHopperToShooter(double outsideVolts, double bottomVolts, double topVolts) {
        return this.run(() -> {
            currentState = IntakeState.HOPPER_TO_SHOOTER;
            io.setOutsideVoltage(-outsideVolts);
            io.setBottomVoltage(-bottomVolts);
            io.setTopVoltage(topVolts);
        }).finallyDo(interrupted -> {
            currentState = IntakeState.IDLE;
            io.setOutsideVoltage(0);
            io.setBottomVoltage(0);
            io.setTopVoltage(0);
        });
    }

    public Command stopCommand() {
        return this.runOnce(() -> {
            currentState = IntakeState.IDLE;
            io.setOutsideVoltage(0);
            io.setBottomVoltage(0);
            io.setTopVoltage(0);
        });
    }

}
