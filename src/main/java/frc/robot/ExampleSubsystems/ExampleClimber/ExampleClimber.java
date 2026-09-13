package frc.robot.ExampleSubsystems.ExampleClimber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ExampleSubsystems.ExampleClimber.ExampleClimberIO.ExampleClimberIOInputsAutoLogged;


public class ExampleClimber extends SubsystemBase
{
    //Handles communication with subsystem's hardware
    private final ExampleClimberIO io;
    
    //Stores sensor and motor information received from the hardware
    private final ExampleClimberIOInputsAutoLogged inputs = new ExampleClimberIOInputsAutoLogged();

    public ExampleClimber(ExampleClimberIO io)
    //Creates the subsystem
    {
    this.io = io;
    }

    /**
     * Runs periodically and updates the subsystem's inputs.
     */
    @Override
    public void periodic() {
        io.updateInputs(inputs);
    }


    /**
     * Example command for controlling the subsystem.
     */
    public Command exampleCommand() {
        return Commands.runOnce(() -> {
            // Tell the subsystem what to do here.
        }, this);
    }


    /**
     * Example getter for accessing subsystem information.
     */
    public double getPosition() {
        return inputs.examplePositionRotations;
    }
}


