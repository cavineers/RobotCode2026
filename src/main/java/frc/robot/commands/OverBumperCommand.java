// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.OverBumperIntake.OverBumperIntake;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class OverBumperCommand extends Command {
  @SuppressWarnings("PMD.UnusedPrivateField")
  private final OverBumperIntake overBumperIntake;

  /**
   * Creates a new ExampleCommand.
   *
   * @param OverBumpersubsystem The subsystem used by this command.
   */
  public OverBumperCommand(OverBumperIntake OverBumpersubsystem) {
    this.overBumperIntake = OverBumpersubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(OverBumpersubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    overBumperIntake.notify(); //this is temporary to make the ugly yellow error go away, since I needed something to do
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
