package frc.robot.subsystems.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Climber extends SubsystemBase {

    /**
     * Climber state machine:
     * "Advance" button: RESTING -> DEPLOYED -> ENGAGED
     * "Retreat" button: ENGAGED -> DEPLOYED -> RESTING
     */
    public enum ClimbState {
        /** All the way down — starting/stored position. */
        RESTING,
        /** All the way up — ready to grab peg. */
        DEPLOYED,
        /** Partially down - robot is hanging on peg. */
        ENGAGED
    }

    private final ClimberIO io;
    private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

    @AutoLogOutput(key = "Climber/ClimbState")
    private ClimbState climbState = ClimbState.RESTING;

    private double kP;
    private double kI;
    private double kD;

    private final LoggedNetworkNumber tuningP = new LoggedNetworkNumber("/Tuning/Climber/kP", ClimberConstants.kP);
    private final LoggedNetworkNumber tuningI = new LoggedNetworkNumber("/Tuning/Climber/kI", ClimberConstants.kI);
    private final LoggedNetworkNumber tuningD = new LoggedNetworkNumber("/Tuning/Climber/kD", ClimberConstants.kD);

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
            io.setPID(kP, kI, kD);
        }

        Logger.processInputs("Climber", inputs);
        Logger.recordOutput("Climber/SetpointRotations", inputs.setpoint);
    }

    // ── State transitions ───────────────────────────────────────────────────

    /**
     * Advance one step forward:
     * RESTING -> DEPLOYED -> ENGAGED (no-op if already ENGAGED)
     */
    public Command advanceCommand() {
        return Commands.runOnce(() -> {
            switch (climbState) {
                case RESTING:
                    io.updateClimberSetpoint(ClimberConstants.kDeployedMotorRotations);
                    climbState = ClimbState.DEPLOYED;
                    break;
                case DEPLOYED:
                    io.updateClimberSetpoint(ClimberConstants.kEngagedMotorRotations);
                    climbState = ClimbState.ENGAGED;
                    break;
                case ENGAGED:
                    // Already at the final climb state — do nothing
                    break;
            }
        }, this);
    }

    /**
     * Retreat one step back:
     * ENGAGED -> DEPLOYED -> RESTING (no-op if already RESTING)
     */
    public Command retreatCommand() {
        return Commands.runOnce(() -> {
            switch (climbState) {
                case ENGAGED:
                    io.updateClimberSetpoint(ClimberConstants.kDeployedMotorRotations);
                    climbState = ClimbState.DEPLOYED;
                    break;
                case DEPLOYED:
                    io.updateClimberSetpoint(ClimberConstants.kRestMotorRotations);
                    climbState = ClimbState.RESTING;
                    break;
                case RESTING:
                    // Already at rest — do nothing
                    break;
            }
        }, this);
    }

    /**
     * If the climber is ENGAGED at the end of autonomous, raise it back up to
     * DEPLOYED so the robot isn't hanging when teleop starts.
     */
    public Command autoEndCommand() {
        return Commands.runOnce(() -> {
            if (climbState == ClimbState.ENGAGED) {
                io.updateClimberSetpoint(ClimberConstants.kDeployedMotorRotations);
                climbState = ClimbState.DEPLOYED;
            }
        }, this);
    }

    public ClimbState getClimbState() {
        return climbState;
    }

    public double getClimberPosition() {
        return inputs.climberPositionRotations;
    }

    public double getSetpoint() {
        return inputs.setpoint;
    }
}