package frc.robot.subsystems.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Climber.ClimberIO.ClimberIOInputsAutoLogged;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Climber extends SubsystemBase {

    /**
     * Climber state machine:
     * "Deploy" button: RESTING -> DEPLOYED -> ENGAGED
     * "Retract" button: ENGAGED -> DEPLOYED -> RESTING
     */
    public enum ClimbState 
    //An enum (short for enumeration) is a Java type used when a variable should only be allowed to have one of a specific set of values.
    {
        /** All the way down — starting/stored position. */
        RESTING,
        /** All the way up — ready to grab peg. */
        DEPLOYED,
        /** Partially down - robot is hanging on peg. */
        ENGAGED
    }

    private final ClimberIO io; 
    //The ClimberIO Interface is used to communicate with the actual Climber Hardware
    //The Climber class handles robot behavior, while the ClimberIO handles the motor/controller-specific implementation
    private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();
    //Stores sensor data received from the climber hardware

    @AutoLogOutput(key = "Climber/ClimbState")
    private ClimbState climbState = ClimbState.RESTING;
    //Logs climbers current state
    //"ClimbState" refers to the enum created above

    //PID Values being used by the climber
    private double kP;
    private double kI;
    private double kD;

    private final LoggedNetworkNumber tuningP = new LoggedNetworkNumber("/Tuning/Climber/kP", ClimberConstants.kP);
    private final LoggedNetworkNumber tuningI = new LoggedNetworkNumber("/Tuning/Climber/kI", ClimberConstants.kI);
    private final LoggedNetworkNumber tuningD = new LoggedNetworkNumber("/Tuning/Climber/kD", ClimberConstants.kD);
    /*LoggedNetworkNumber allows PID values to be changed through AdvantageScope/Network Tables while the robot is running */
    //The values from ClimberConstants are used as the inital values

    public Climber(ClimberIO io) {
        this.io = io;
    }
    //Creates the climber subsystem
    //io implementation responsible for communicating with the climber motor and sensors

    @Override
    public void periodic()
    //Runs periodically while the robot is enabled
    //Updates sensor inputs
    //Check for any change in PID Values
    //Records info for logging 
    {
        io.updateInputs(inputs);
        //reads the lates sensor input from the climber hardware

        if (kP != tuningP.get() || kI != tuningI.get() || kD != tuningD.get()) {
            kP = tuningP.get();
            kI = tuningI.get();
            kD = tuningD.get();
            io.setPID(kP, kI, kD);
            //Send the updated PID values to the motor controller
        }
        //Check if any PID values have been changed during tuning
        //Only update the motor controller if a value has actually been changed

        Logger.processInputs("Climber", (LoggableInputs) inputs);
        //Log sensor inputs for debugging and analysis
        Logger.recordOutput("Climber/SetpointRotations", inputs.setpoint);
        //Log current motor position setpoint
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
                    //Move from resting position to deployed position
                    io.updateClimberSetpoint(ClimberConstants.kDeployedMotorRotations);
                    climbState = ClimbState.DEPLOYED;
                    break;
                case DEPLOYED:
                    //Move from deployed position to engaged position
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
                    //Move from engaged position back to deployed position
                    io.updateClimberSetpoint(ClimberConstants.kDeployedMotorRotations);
                    climbState = ClimbState.DEPLOYED;
                    break;
                case DEPLOYED:
                    //Move from deployed position to resting position
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
                //Move back to deployed position
                io.updateClimberSetpoint(ClimberConstants.kDeployedMotorRotations);
                climbState = ClimbState.DEPLOYED;
            }
        }, this);
    }

    public ClimbState getClimbState() 
    //return climber's current state
    {
        return climbState;
    }

    public double getClimberPosition() 
    //return climber's current physical position, measured in rotations
    {
        return inputs.climberPositionRotations;
    }

    public double getSetpoint() 
    //return climber's current target position in rotations
    {
        return inputs.setpoint;
    }
}